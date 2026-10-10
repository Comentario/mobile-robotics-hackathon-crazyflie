"""RViz presentation in a dedicated, start-relative frame; no drone commands."""
import math

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


class RvizPublisher:
    FRAME = 'cf2_display'

    def __init__(self, node):
        self.node = node
        self.tf = TransformBroadcaster(node)
        self.map_pub = node.create_publisher(OccupancyGrid, '/cf2/presentation/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.trail_pub = node.create_publisher(Path, '/cf2/presentation/trail', 10)
        self.plan_pub = node.create_publisher(Path, '/cf2/presentation/plan', 10)
        self.markers_pub = node.create_publisher(MarkerArray, '/cf2/presentation/markers', 10)

    def publish(self, s):
        if s['origin'] is None or s['start'] is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        sx, sy = s['start']
        def point(x, y, z=0.):
            return Point(x=float(x-sx), y=float(y-sy), z=float(z))
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, self.FRAME
        grid.info.resolution = float(s['resolution'])
        grid.info.width, grid.info.height = s['width'], s['height']
        grid.info.origin.position = point(*s['origin'])
        grid.info.origin.orientation.w = 1.
        grid.data = s['presentation_data']
        self.map_pub.publish(grid)
        for pub, points in [(self.trail_pub, s['trail']),
                            (self.plan_pub, (s['navigation'] or {}).get('path', []))]:
            path = Path(header=grid.header)
            for coordinates in points:
                p = PoseStamped(header=grid.header)
                p.pose.position = point(*coordinates[:2], .025)
                p.pose.orientation.w = 1.
                path.poses.append(p)
            pub.publish(path)
        markers = MarkerArray()
        clear = Marker(header=grid.header, action=Marker.DELETEALL)
        markers.markers.append(clear)
        def marker(name, kind, color, size):
            m = Marker(header=grid.header, ns=name, id=0, type=kind, action=Marker.ADD)
            m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.scale.x, m.scale.y, m.scale.z = size
            markers.markers.append(m)
            return m
        home = marker('Start position', Marker.CYLINDER, (1., .72, .12, 1.), (.18, .18, .025))
        home.pose.position.z = .02
        label = marker('Start label', Marker.TEXT_VIEW_FACING, (1., .83, .35, 1.), (0., 0., .12))
        label.pose.position.x, label.pose.position.y, label.pose.position.z = -.1, -.22, .08
        label.text = 'START (0, 0)'
        hits = marker('Recent returns', Marker.SPHERE_LIST, (1., .55, .12, .65), (.065, .065, .065))
        hits.points = [point(x, y, .035) for x, y in s['recent_hits']]
        persistent = marker('Persistent observations', Marker.CUBE_LIST, (.22, .68, .95, .9),
                            (s['resolution'], s['resolution'], .07))
        for i, value in enumerate(s['presentation_data']):
            if value == 100:
                x = s['origin'][0]+(i % s['width']+.5)*s['resolution']
                y = s['origin'][1]+(i // s['width']+.5)*s['resolution']
                persistent.points.append(point(x, y, .04))
        pose = s['pose']
        fresh = s['age_seconds'] is not None and s['age_seconds'] <= 1
        if pose:
            x, y, z = (pose['stateEstimate.'+axis] for axis in 'xyz')
            yaw = math.radians(pose['stateEstimate.yaw'])
            transform = TransformStamped(header=grid.header, child_frame_id='cf2_display_body')
            transform.transform.translation.x = float(x-sx)
            transform.transform.translation.y = float(y-sy)
            transform.transform.translation.z = float(z)
            transform.transform.rotation.z = math.sin(yaw/2)
            transform.transform.rotation.w = math.cos(yaw/2)
            self.tf.sendTransform(transform)
            body = marker('Crazyflie', Marker.SPHERE, (.86, .94, 1., 1.), (.14, .14, .05))
            body.pose.position = point(x, y, max(0., z))
            heading = marker('Heading', Marker.ARROW, (.95, .26, .32, 1.), (.025, .065, .085))
            heading.points = [point(x, y, max(0., z)+.04),
                              point(x+.25*math.cos(yaw), y+.25*math.sin(yaw), max(0., z)+.04)]
            if fresh:
                for direction, offset, color in [
                    ('front', 0., (.2, .95, .45, .85)),
                    ('left', math.pi/2, (.15, .8, 1., .85)),
                    ('back', math.pi, (1., .75, .2, .85)),
                    ('right', -math.pi/2, (.85, .4, 1., .85))]:
                    raw = s['ranges_mm'].get('range.'+direction)
                    if raw is None or not math.isfinite(raw) or not 50 <= raw <= 4000:
                        continue
                    beam = marker(direction.title()+' ToF', Marker.LINE_LIST, color, (.012, 0., 0.))
                    beam.points = [point(x, y, max(0., z)), point(
                        x+raw/1000*math.cos(yaw+offset), y+raw/1000*math.sin(yaw+offset), max(0., z))]
            status = marker('Live status', Marker.TEXT_VIEW_FACING, (.8, .88, .98, 1.), (0., 0., .10))
            status.pose.position = point(x, y+.32, max(0., z)+.12)
            nav = s['navigation'] or {}
            status.text = (f"CF2 | {nav.get('state', 'SENSING') if fresh else 'STALE DATA'}\n"
                           f"X {x-sx:+.2f} m  Y {y-sy:+.2f} m  Z {z:.2f} m")
        session = marker('Session reference', Marker.TEXT_VIEW_FACING, (.65, .73, .84, 1.), (0., 0., .10))
        session.pose.position.x, session.pose.position.y = -1.5, 1.5
        session.text = (f"CF2 LIVE PERCEPTION | {s['session_id']}\n"
                        'Local Flow/EKF frame | shared-frame alignment pending\n'
                        'Blue: persistent observations | orange: recent returns')
        self.markers_pub.publish(markers)
