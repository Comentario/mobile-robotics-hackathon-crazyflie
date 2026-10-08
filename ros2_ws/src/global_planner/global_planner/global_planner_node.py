"""ROS 2 global planner wrapper for TP-27.

Modes:
  direct: straight path from current position to goal, without a map.
  astar: A* over /hk/map using the TP-27 binary occupancy policy.

Inputs:
  /hk/map                   nav_msgs/OccupancyGrid
  /hk/state                 nav_msgs/Odometry
  /hk/goal                  geometry_msgs/PoseStamped

Outputs:
  /hk/path                  nav_msgs/Path
  /hk/planner_status        hackathon_interfaces/PlannerStatus

TP-28 adds configurable occupancy costs and unknown-space policy.
TP-29 adds replanning triggers.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from hackathon_interfaces.msg import PlannerStatus

from .astar import (
    astar,
    binary_cost_map,
    cell_to_world,
    direct_path,
    world_to_cell,
)


class GlobalPlanner(Node):
    """Generate a direct or A* path toward the current mission goal."""

    def __init__(self):
        super().__init__('global_planner')

        self._declare_parameters()
        self._read_parameters()

        self.position = None
        self.goal = None
        self.map_msg = None

        self.plan_requested = False
        self.replan_count = 0

        self.path_pub = self.create_publisher(
            Path,
            '/hk/path',
            10,
        )

        self.status_pub = self.create_publisher(
            PlannerStatus,
            '/hk/planner_status',
            10,
        )

        self.create_subscription(
            Odometry,
            '/hk/state',
            self._state_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            '/hk/goal',
            self._goal_callback,
            10,
        )

        self.create_subscription(
            OccupancyGrid,
            '/hk/map',
            self._map_callback,
            10,
        )

        self.steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        self.create_timer(
            1.0 / self.planner_rate,
            self._planning_tick,
            clock=self.steady_clock,
        )

        self._publish_status(
            PlannerStatus.IDLE,
            0.0,
            '',
        )

        self.get_logger().info(
            f'TP-27 global planner ready in {self.planner_mode} mode'
        )

    def _declare_parameters(self):
        """Declare planner parameters."""
        self.declare_parameter('planner_mode', 'direct')
        self.declare_parameter('planner_rate', 2.0)
        self.declare_parameter('connectivity', 8)
        self.declare_parameter('planning_timeout', 0.2)
        self.declare_parameter('direct_spacing', 0.3)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('map_frame', 'map')

    def _read_parameters(self):
        """Read and validate planner parameters."""
        self.planner_mode = str(
            self.get_parameter('planner_mode').value
        )

        self.planner_rate = float(
            self.get_parameter('planner_rate').value
        )

        self.connectivity = int(
            self.get_parameter('connectivity').value
        )

        self.planning_timeout = float(
            self.get_parameter('planning_timeout').value
        )

        self.direct_spacing = float(
            self.get_parameter('direct_spacing').value
        )

        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value
        )

        self.map_frame = str(
            self.get_parameter('map_frame').value
        )

        if self.planner_mode not in ('direct', 'astar'):
            raise ValueError(
                'planner_mode must be "direct" or "astar"'
            )

        if self.planner_rate <= 0.0:
            raise ValueError('planner_rate must be positive')

        if self.connectivity not in (4, 8):
            raise ValueError('connectivity must be 4 or 8')

        if self.planning_timeout <= 0.0:
            raise ValueError(
                'planning_timeout must be positive'
            )

        if self.direct_spacing <= 0.0:
            raise ValueError(
                'direct_spacing must be positive'
            )

        if not 0 <= self.occupied_threshold <= 100:
            raise ValueError(
                'occupied_threshold must be in [0, 100]'
            )

    def _state_callback(self, msg):
        """Store the latest vehicle position."""
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        if not all(math.isfinite(value) for value in values):
            return

        self.position = tuple(
            float(value)
            for value in values
        )

    def _goal_callback(self, msg):
        """Store a new goal and request planning."""
        if msg.header.frame_id not in ('', self.map_frame):
            self.get_logger().warn(
                f'Ignoring goal in frame "{msg.header.frame_id}"; '
                f'expected "{self.map_frame}"'
            )
            return

        values = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn(
                'Ignoring goal with non-finite coordinates'
            )
            return

        new_goal = tuple(
            float(value)
            for value in values
        )

        if new_goal != self.goal:
            self.goal = new_goal
            self.plan_requested = True

    def _map_callback(self, msg):
        """Store the latest occupancy grid."""
        self.map_msg = msg

        # A map arriving can satisfy a previously blocked initial plan.
        if (
            self.planner_mode == 'astar'
            and self.goal is not None
            and self.plan_requested
        ):
            self.plan_requested = True

    def _planning_tick(self):
        """Execute one requested plan."""
        if not self.plan_requested:
            return

        if self.position is None or self.goal is None:
            return

        if self.planner_mode == 'direct':
            self._plan_direct()
            return

        if self.map_msg is None:
            self._publish_status(
                PlannerStatus.NO_MAP,
                0.0,
                'no_map',
            )
            return

        self._plan_astar()

    def _plan_direct(self):
        """Publish a straight path without using a map."""
        start_time = time.monotonic()

        points = direct_path(
            start=self.position,
            goal=self.goal,
            spacing=self.direct_spacing,
        )

        elapsed = time.monotonic() - start_time

        self._publish_world_path(points)

        self._publish_status(
            PlannerStatus.OK,
            elapsed,
            '',
        )

        self.plan_requested = False

    def _plan_astar(self):
        """Run A* on the latest occupancy grid."""
        start_time = time.monotonic()

        msg = self.map_msg
        info = msg.info

        if info.resolution <= 0.0:
            self._publish_failure(
                PlannerStatus.NO_MAP,
                'invalid_map_resolution',
                start_time,
            )
            return

        if info.width <= 0 or info.height <= 0:
            self._publish_failure(
                PlannerStatus.NO_MAP,
                'empty_map',
                start_time,
            )
            return

        origin = info.origin.position

        start_cell = world_to_cell(
            x=self.position[0],
            y=self.position[1],
            origin_x=origin.x,
            origin_y=origin.y,
            resolution=info.resolution,
            width=info.width,
            height=info.height,
        )

        goal_cell = world_to_cell(
            x=self.goal[0],
            y=self.goal[1],
            origin_x=origin.x,
            origin_y=origin.y,
            resolution=info.resolution,
            width=info.width,
            height=info.height,
        )

        if start_cell is None:
            self._publish_failure(
                PlannerStatus.NO_PATH,
                'start_outside_map',
                start_time,
            )
            return

        if goal_cell is None:
            self._publish_failure(
                PlannerStatus.NO_PATH,
                'goal_outside_map',
                start_time,
            )
            return

        try:
            cost_map = binary_cost_map(
                data=msg.data,
                width=info.width,
                height=info.height,
                occupied_threshold=self.occupied_threshold,
            )
        except ValueError as exc:
            self._publish_failure(
                PlannerStatus.NO_MAP,
                str(exc),
                start_time,
            )
            return

        cells = astar(
            cost_map=cost_map,
            start=start_cell,
            goal=goal_cell,
            connectivity=self.connectivity,
            timeout=self.planning_timeout,
        )

        elapsed = time.monotonic() - start_time

        if cells is None:
            status = PlannerStatus.NO_PATH
            reason = 'no_path'

            if elapsed >= self.planning_timeout:
                status = PlannerStatus.TIMEOUT
                reason = 'planning_timeout'

            self._publish_status(
                status,
                elapsed,
                reason,
            )

            self._publish_world_path([])
            self.plan_requested = False
            return

        points = []

        for cell in cells:
            x, y = cell_to_world(
                cell=cell,
                origin_x=origin.x,
                origin_y=origin.y,
                resolution=info.resolution,
            )

            points.append(
                (
                    x,
                    y,
                    self.goal[2],
                )
            )

        self._publish_world_path(points)

        self._publish_status(
            PlannerStatus.OK,
            elapsed,
            '',
        )

        self.plan_requested = False

    def _publish_failure(
        self,
        status,
        reason,
        start_time,
    ):
        """Publish a planner failure and clear any stale path."""
        elapsed = time.monotonic() - start_time

        self._publish_world_path([])

        self._publish_status(
            status,
            elapsed,
            reason,
        )

        self.plan_requested = False

    def _publish_world_path(self, points):
        """Publish a nav_msgs/Path in the map frame."""
        msg = Path()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for x, y, z in points:
            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def _publish_status(
        self,
        status,
        plan_time,
        reason,
    ):
        """Publish planner state."""
        msg = PlannerStatus()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.status = int(status)
        msg.replan_count = int(self.replan_count)
        msg.plan_time = float(plan_time)
        msg.reason = str(reason)

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = GlobalPlanner()

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
