"""RViz presentation in a dedicated, start-relative frame; no drone commands."""
import math
from copy import deepcopy

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


class RvizPublisher:
    FRAME = 'cf2_display'
    PALETTE = [(0., .48, .82, 1.), (1., .48, .05, 1.), (.62, .25, .83, 1.),
               (.08, .65, .35, 1.), (.90, .18, .36, 1.)]

    def __init__(self, node):
        self.node = node
        self.tf = TransformBroadcaster(node)
        self.map_pub = node.create_publisher(OccupancyGrid, '/cf2/presentation/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.trail_pub = node.create_publisher(Path, '/cf2/presentation/trail', 10)
        self.plan_pub = node.create_publisher(Path, '/cf2/presentation/plan', 10)
        self.markers_pub = node.create_publisher(MarkerArray, '/cf2/presentation/markers', 10)
        self.layer_pubs = {}

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
        for layer in s.get('layers', []):
            index = layer['index']
            if index not in self.layer_pubs:
                self.layer_pubs[index] = self.node.create_publisher(OccupancyGrid,
                    f'/cf2/presentation/layer_{index}',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            message = OccupancyGrid(header=grid.header, info=deepcopy(grid.info), data=layer['data'])
            message.info.origin.position.z = float(layer['height_m'])
            self.layer_pubs[index].publish(message)
        for pub, points in [(self.trail_pub, s['trail']),
                            (self.plan_pub, (s['navigation'] or {}).get('path', [])[:5])]:
            path = Path(header=grid.header)
            for coordinates in points:
                p = PoseStamped(header=grid.header)
                p.pose.position = point(*coordinates[:2], .12)
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
            m.scale.x, m.scale.y, m.scale.z = map(float, size)
            markers.markers.append(m)
            return m
        home = marker('Start position', Marker.CYLINDER, (1., .72, .12, 1.), (.13, .13, .025))
        home.pose.position.z = .02
        hits = marker('Recent returns', Marker.SPHERE_LIST, (1., .55, .12, .65), (.065, .065, .065))
        hits.points = [point(x, y, .035) for x, y in s['recent_hits']]
        # Coplanar triangle tiles avoid cube side edges and z-fighting seams.
        observed = marker('Observed free space', Marker.TRIANGLE_LIST, (.97, .98, 1., 1.), (1., 1., 1.))
        persistent = marker('Persistent observations', Marker.TRIANGLE_LIST, (.04, .05, .07, 1.), (1., 1., 1.))
        for i, value in enumerate(s['presentation_data']):
            if value in (0, 100):
                x = s['origin'][0]+(i % s['width']+.5)*s['resolution']
                y = s['origin'][1]+(i // s['width']+.5)*s['resolution']
                half = s['resolution']/2
                z = .04 if value == 100 else -.005
                corners = [point(x-half,y-half,z), point(x+half,y-half,z),
                           point(x+half,y+half,z), point(x-half,y+half,z)]
                (persistent if value == 100 else observed).points.extend(
                    [corners[0],corners[1],corners[2],corners[0],corners[2],corners[3]])
        for layer in s.get('layers', []):
            index, height = layer['index'], layer['height_m']
            color = self.PALETTE[index % len(self.PALETTE)]
            # Nested colored cell borders show overlapping obstacles at several
            # discrete heights while preserving the black occupancy footprint.
            contour = marker(f'Z {height:.2f} m obstacle outlines', Marker.LINE_LIST,
                             color, (.004, 0., 0.))
            half = s['resolution']/2-.002-index*.003
            for i, value in enumerate(layer['presentation_data']):
                if value != 100:
                    continue
                x = s['origin'][0]+(i % s['width']+.5)*s['resolution']
                y = s['origin'][1]+(i // s['width']+.5)*s['resolution']
                corners = [(x-half,y-half), (x+half,y-half), (x+half,y+half), (x-half,y+half)]
                for a, b in zip(corners, corners[1:]+corners[:1]):
                    contour.points.extend([point(*a, .08), point(*b, .08)])
            trace = marker(f'Z {height:.2f} m flight trail', Marker.LINE_STRIP,
                           color, (.009, 0., 0.))
            trace.points = [point(*p[:2], .125+index*.001) for p in layer['trail']]
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
            body = marker('Crazyflie', Marker.SPHERE,
                          (.05, .45, .77, 1.) if fresh else (.45, .48, .53, 1.), (.16, .16, .035))
            body.pose.position = point(x, y, .08)
            heading = marker('Heading', Marker.ARROW, (.95, .26, .32, 1.), (.025, .065, .085))
            heading.points = [point(x, y, .10),
                              point(x+.25*math.cos(yaw), y+.25*math.sin(yaw), .10)]
            if fresh:
                for direction, offset, color in [
                    ('front', 0., (.2, .95, .45, .85)),
                    ('left', math.pi/2, (.15, .8, 1., .85)),
                    ('back', math.pi, (1., .75, .2, .85)),
                    ('right', -math.pi/2, (.85, .4, 1., .85))]:
                    raw = s['ranges_mm'].get('range.'+direction)
                    if raw is None or not math.isfinite(raw) or not 50 <= raw <= 4000:
                        continue
                    distance = min(raw/1000, s.get('free_radius_m') or s.get('mapping_radius_m', .8))
                    beam = marker(direction.title()+' ToF', Marker.LINE_LIST, color, (.012, 0., 0.))
                    beam.points = [point(x, y, .08), point(
                        x+distance*math.cos(yaw+offset), y+distance*math.sin(yaw+offset), .08)]
        coverage = (s['navigation'] or {}).get('coverage')
        if coverage:
            width, length = coverage['area']
            cx, cy = (s['navigation'] or {}).get('start') or s['start']
            unknown = marker('Unknown survey area', Marker.CUBE, (.65, .83, .95, 1.), (width, length, .004))
            unknown.pose.position = point(cx, cy, -.02)
            boundary = marker('Survey boundary', Marker.LINE_STRIP, (.18, .32, .52, 1.), (.025, 0., 0.))
            boundary.points = [point(cx+x, cy+y, .015) for x, y in
                               [(-width/2,-length/2), (width/2,-length/2),
                                (width/2,length/2), (-width/2,length/2), (-width/2,-length/2)]]
            local_target = (s['navigation'] or {}).get('local_target')
            if local_target is not None:
                target = marker('Local movement target', Marker.CYLINDER, (.64, .26, .85, 1.), (.09, .09, .015))
                target.pose.position = point(*local_target[:2], .03)
        # No world-space text: billboard fonts distort in the WSL top-down view.
        # Status, freshness and coverage are reported by the viewer terminal.
        self.markers_pub.publish(markers)
