"""ROS 2 path processor for TP-30.

Inputs:
  /hk/path                  nav_msgs/Path
  /hk/map                   nav_msgs/OccupancyGrid

Output:
  /hk/path/processed        nav_msgs/Path

Processing:
  remove collinear points
  safe line-of-sight shortcut when a map is available
  densify to max_waypoint_spacing
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from .simplify import (
    densify,
    occupancy_segment_is_free,
    remove_collinear,
    shortcut,
)


class PathProcessor(Node):
    """Convert discrete planner paths into executable path references."""

    def __init__(self):
        super().__init__('path_processor')

        self.declare_parameter(
            'max_waypoint_spacing',
            0.3,
        )

        self.declare_parameter(
            'occupied_threshold',
            50,
        )

        self.declare_parameter(
            'unknown_is_free',
            False,
        )

        self.max_waypoint_spacing = float(
            self.get_parameter(
                'max_waypoint_spacing'
            ).value
        )

        self.occupied_threshold = int(
            self.get_parameter(
                'occupied_threshold'
            ).value
        )

        self.unknown_is_free = bool(
            self.get_parameter(
                'unknown_is_free'
            ).value
        )

        if self.max_waypoint_spacing <= 0.0:
            raise ValueError(
                'max_waypoint_spacing must be positive'
            )

        if not 0 <= self.occupied_threshold <= 100:
            raise ValueError(
                'occupied_threshold must be in [0, 100]'
            )

        self.map_msg = None

        self.path_pub = self.create_publisher(
            Path,
            '/hk/path/processed',
            10,
        )

        self.create_subscription(
            Path,
            '/hk/path',
            self._path_callback,
            10,
        )

        self.create_subscription(
            OccupancyGrid,
            '/hk/map',
            self._map_callback,
            10,
        )

        self.get_logger().info(
            'TP-30 path processor ready: '
            '/hk/path -> /hk/path/processed'
        )

    def _map_callback(self, msg):
        """Store the latest occupancy grid."""
        self.map_msg = msg

    def _segment_is_free(self, start, end):
        """Check map line of sight for one shortcut candidate."""
        if self.map_msg is None:
            return False

        msg = self.map_msg
        info = msg.info

        return occupancy_segment_is_free(
            start=start,
            end=end,
            data=msg.data,
            width=info.width,
            height=info.height,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            resolution=info.resolution,
            occupied_threshold=self.occupied_threshold,
            unknown_is_free=self.unknown_is_free,
        )

    def _path_callback(self, msg):
        """Simplify and publish one planner path."""
        points = []

        for pose in msg.poses:
            values = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )

            if not all(
                math.isfinite(value)
                for value in values
            ):
                self.get_logger().warn(
                    'Ignoring path containing non-finite coordinates'
                )
                return

            points.append(
                tuple(float(value) for value in values)
            )

        if not points:
            self._publish_path(
                [],
                msg.header.frame_id,
            )
            return

        processed = remove_collinear(points)

        if self.map_msg is not None:
            processed = shortcut(
                processed,
                self._segment_is_free,
            )

        processed = densify(
            processed,
            self.max_waypoint_spacing,
        )

        self._publish_path(
            processed,
            msg.header.frame_id,
        )

    def _publish_path(self, points, frame_id):
        """Publish processed points as nav_msgs/Path."""
        msg = Path()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id or 'map'

        for x, y, z in points:
            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PathProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
