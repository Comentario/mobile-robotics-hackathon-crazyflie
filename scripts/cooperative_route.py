"""Serve the current mission leg from an offline route, without radio commands.

Consumes /hk/goal and serves /hk/path/processed. Synthetic routes require an
explicit flag. Start the mission only after frame/clearance checks on hardware.
"""
import argparse
import json
import math
from pathlib import Path


def validate_route(result, allow_synthetic=False):
    if result.get('simulation') and not allow_synthetic:
        raise ValueError('Synthetic route requires --allow-synthetic for software testing')
    if result.get('frame_id') != 'map' or not result.get('modified') or not result.get('legs'):
        raise ValueError('No complete route for the requested mission; proposal files are not substituted')
    legs = result['legs']
    previous = None
    for index, leg in enumerate(legs):
        points = leg['points']
        if leg['waypoint_index'] != index or not points:
            raise ValueError('Invalid ordered mission legs')
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Invalid route coordinates')
        if previous is not None and math.dist(points[0], previous) > 1e-6:
            raise ValueError('Discontinuous mission legs')
        previous = points[-1]
    return legs


def find_leg(legs, goal):
    return next((leg for leg in legs if math.dist(leg['points'][-1], goal) <= 1e-4), None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--allow-synthetic', action='store_true')
    args = parser.parse_args()
    try:
        result = json.loads(args.route.read_text())
        legs = validate_route(result, args.allow_synthetic)
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, f'Route not enabled: {error}\n')

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from hackathon_interfaces.msg import PlannerStatus

    class RouteServer(Node):
        def __init__(self):
            super().__init__('cooperative_route')
            self.goal, self.goal_stamp = None, None
            self.path_pub = self.create_publisher(RosPath, '/hk/path/processed', 10)
            self.status_pub = self.create_publisher(PlannerStatus, '/hk/planner_status', 10)
            self.create_subscription(PoseStamped, '/hk/goal', self.on_goal, 10)
            self.create_timer(.1, self.tick)
            self.get_logger().info(f'{len(legs)} mission legs ready; no radio or mission-start calls')

        def on_goal(self, msg):
            p = msg.pose.position
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now = self.get_clock().now().nanoseconds/1e9
            if (msg.header.frame_id != 'map' or not 0 <= now-stamp <= .5
                    or not all(math.isfinite(v) for v in (p.x, p.y, p.z))):
                return
            self.goal = (p.x, p.y, p.z)
            self.goal_stamp = stamp

        def tick(self):
            stamp = self.get_clock().now().to_msg()
            status = PlannerStatus()
            status.header.stamp, status.header.frame_id = stamp, 'map'
            if self.goal is None:
                status.status, status.reason = PlannerStatus.IDLE, 'waiting_for_goal'
                self.status_pub.publish(status)
                return
            leg = find_leg(legs, self.goal)
            msg = RosPath()
            msg.header.stamp, msg.header.frame_id = stamp, 'map'
            if leg is None:
                status.status, status.reason = PlannerStatus.NO_PATH, 'goal_not_in_planned_mission'
            else:
                status.status, status.reason = PlannerStatus.OK, f'waypoint_{leg["waypoint_index"]+1}'
                for x, y, z in leg['points']:
                    p = PoseStamped()
                    p.header = msg.header
                    p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
                    p.pose.orientation.w = 1.
                    msg.poses.append(p)
            # Navigator independently checks fresh mission goals and measured position.
            self.path_pub.publish(msg)
            self.status_pub.publish(status)

    rclpy.init()
    node = RouteServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
