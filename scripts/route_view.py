#!/usr/bin/env python3
"""RViz view of a CF1 cooperative flight. Visualization only, never flight commands.

Publishes the CF2 map and the planned route from a route JSON (latched), and the
trajectory actually flown, accumulated from /hk/state:
  /hk/shared_map/raw, /hk/shared_map/inflated   nav_msgs/OccupancyGrid
  /hk/path/modified                             nav_msgs/Path (planned route)
  /hk/trajectory                                nav_msgs/Path (flown)
Without --route only the flown trajectory is published.
"""
import argparse
import json
import math
from pathlib import Path

from plan_shared_map import transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path)
    parser.add_argument('--min-step', type=float, default=.02,
                        help='Minimum XYZ distance in m between stored trajectory points')
    args = parser.parse_args()
    route = json.loads(args.route.read_text()) if args.route else None

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    class RouteView(Node):
        def __init__(self):
            super().__init__('route_view')
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.trajectory = RosPath()
            self.trajectory.header.frame_id = 'map'
            self.last = None
            self.trajectory_pub = self.create_publisher(RosPath, '/hk/trajectory', 10)
            self.create_subscription(Odometry, '/hk/state', self.on_state, 10)
            if route is not None:
                self.publish_route(latched)
            self.create_timer(.5, self.publish_trajectory)

        def pose(self, x, y, z, stamp):
            msg = PoseStamped()
            msg.header.frame_id, msg.header.stamp = 'map', stamp
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
            msg.pose.orientation.w = 1.0
            return msg

        def publish_route(self, qos):
            stamp = self.get_clock().now().to_msg()
            path = RosPath()
            path.header.frame_id, path.header.stamp = 'map', stamp
            path.poses = [self.pose(*p, stamp) for p in route['modified']]
            self.route_pub = self.create_publisher(RosPath, '/hk/path/modified', qos)
            self.route_pub.publish(path)
            grid, tf = route['grid'], route['transform']
            origin = transform([*grid['origin'], 0.0], tf)
            self.map_pubs = []
            for key in ('raw', 'inflated'):
                msg = OccupancyGrid()
                msg.header.frame_id, msg.header.stamp = 'map', stamp
                msg.info.width, msg.info.height = grid['width'], grid['height']
                msg.info.resolution = float(grid['resolution'])
                msg.info.origin.position.x, msg.info.origin.position.y = origin[:2]
                msg.info.origin.orientation.z = math.sin(tf[2]/2)
                msg.info.origin.orientation.w = math.cos(tf[2]/2)
                msg.data = grid[key]
                pub = self.create_publisher(OccupancyGrid, f'/hk/shared_map/{key}', qos)
                pub.publish(msg)
                self.map_pubs.append(pub)

        def on_state(self, msg):
            if msg.header.frame_id != 'map':
                return
            p = msg.pose.pose.position
            point = (p.x, p.y, p.z)
            if not all(math.isfinite(v) for v in point):
                return
            if self.last is None or math.dist(point, self.last) >= args.min_step:
                self.trajectory.poses.append(self.pose(*point, msg.header.stamp))
                self.last = point

        def publish_trajectory(self):
            self.trajectory.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_pub.publish(self.trajectory)

    rclpy.init()
    node = RouteView()
    node.get_logger().info('Visualization only: /hk/trajectory'
                           + (', /hk/path/modified, /hk/shared_map/{raw,inflated}' if route else ''))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
