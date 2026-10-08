"""ROS 2 wrapper for the TP-32 direct position follower.

MVP input:
  /hk/state                 nav_msgs/Odometry
  /hk/goal                  geometry_msgs/PoseStamped

Output:
  /hk/cmd/nav               hackathon_interfaces/Setpoint

The direct-goal input is temporary for the MVP. Later planning TPs can replace
it with /hk/path/processed without changing the command interface.

Only safety_supervisor may publish /hk/cmd/safe.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from hackathon_interfaces.msg import Setpoint

from .follower import direct_position_setpoint, yaw_towards


class LocalNavigator(Node):
    """Generate bounded position setpoints toward the active waypoint."""

    def __init__(self):
        super().__init__('local_navigator')

        self._declare_parameters()
        self._read_parameters()

        self.position = None
        self.current_yaw = 0.0
        self.goal = None
        self.last_goal_monotonic = None

        self.command_pub = self.create_publisher(
            Setpoint,
            '/hk/cmd/nav',
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

        self.steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        self.create_timer(
            1.0 / self.control_rate,
            self._control_tick,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            'TP-32 direct position follower ready: '
            '/hk/state + /hk/goal -> /hk/cmd/nav'
        )

    def _declare_parameters(self):
        """Declare follower parameters."""
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('nominal_speed', 0.3)
        self.declare_parameter('z_nominal', 0.5)
        self.declare_parameter('goal_timeout', 0.5)
        self.declare_parameter('map_frame', 'map')

    def _read_parameters(self):
        """Read and validate follower parameters."""
        self.control_rate = float(
            self.get_parameter('control_rate').value
        )
        self.nominal_speed = float(
            self.get_parameter('nominal_speed').value
        )
        self.z_nominal = float(
            self.get_parameter('z_nominal').value
        )
        self.goal_timeout = float(
            self.get_parameter('goal_timeout').value
        )
        self.map_frame = str(
            self.get_parameter('map_frame').value
        )

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        if self.nominal_speed <= 0.0:
            raise ValueError('nominal_speed must be positive')

        if self.goal_timeout <= 0.0:
            raise ValueError('goal_timeout must be positive')

    def _state_callback(self, msg):
        """Store the latest normalized vehicle state."""
        self.position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )

        q = msg.pose.pose.orientation

        self.current_yaw = math.atan2(
            2.0 * (
                q.w * q.z
                + q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
            ),
        )

    def _goal_callback(self, msg):
        """Store the latest direct waypoint."""
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
                'Ignoring goal with non-finite position'
            )
            return

        self.goal = tuple(float(value) for value in values)
        self.last_goal_monotonic = time.monotonic()

    def _control_tick(self):
        """Publish one bounded navigation proposal."""
        if self.position is None or self.goal is None:
            return

        if self.last_goal_monotonic is None:
            return

        goal_age = (
            time.monotonic()
            - self.last_goal_monotonic
        )

        if goal_age > self.goal_timeout:
            return

        max_step = (
            self.nominal_speed
            / self.control_rate
        )

        target = direct_position_setpoint(
            position=self.position,
            goal=self.goal,
            max_step=max_step,
            z_nominal=self.z_nominal,
        )

        yaw = yaw_towards(
            position=self.position,
            target=self.goal,
            current_yaw=self.current_yaw,
        )

        msg = Setpoint()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.mode = Setpoint.MODE_POSITION

        msg.position.x = target[0]
        msg.position.y = target[1]
        msg.position.z = target[2]

        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0

        msg.yaw = float(yaw)

        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LocalNavigator()

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
