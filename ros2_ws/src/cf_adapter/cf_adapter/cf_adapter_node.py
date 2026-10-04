"""ROS 2 adapter between the Hackathon API and Crazyswarm2.

This package is the only project component allowed to depend directly on
Crazyswarm2-specific topics, services, and message types.
"""

from functools import partial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, Trigger

from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import (
    Land as CfLand,
    NotifySetpointsStop,
    Takeoff as CfTakeoff,
)

from hackathon_interfaces.msg import MultiRange, Setpoint
from hackathon_interfaces.srv import (
    Land as HkLand,
    Takeoff as HkTakeoff,
)

from .mapping import (
    seconds_to_duration_parts,
    topic,
    yaw_to_quaternion,
)


class CrazyflieAdapter(Node):
    """Expose the project hardware API and translate it to Crazyswarm2."""

    def __init__(self):
        super().__init__('cf_adapter')

        self.declare_parameter('robot_name', 'cf231')
        self.declare_parameter('group_mask', 0)
        self.declare_parameter('land_height', 0.0)
        self.declare_parameter('streaming_valid_ms', 100)

        self.robot_name = (
            self.get_parameter('robot_name')
            .get_parameter_value()
            .string_value
        )

        self.group_mask = (
            self.get_parameter('group_mask')
            .get_parameter_value()
            .integer_value
        )

        self.land_height = (
            self.get_parameter('land_height')
            .get_parameter_value()
            .double_value
        )

        self.streaming_valid_ms = (
            self.get_parameter('streaming_valid_ms')
            .get_parameter_value()
            .integer_value
        )

        # Project-facing publishers.
        # Data will be populated once the real Crazyflie state and sensor
        # endpoints are confirmed.
        self.odom_pub = self.create_publisher(
            Odometry,
            '/hk/adapter/odom',
            10,
        )

        self.ranges_pub = self.create_publisher(
            MultiRange,
            '/hk/adapter/ranges',
            10,
        )

        # Project-facing command subscriber.
        self.command_sub = self.create_subscription(
            Setpoint,
            '/hk/cmd/safe',
            self._command_callback,
            10,
        )

        # Project-facing services.
        self.takeoff_srv = self.create_service(
            HkTakeoff,
            '/hk/takeoff',
            self._takeoff_callback,
        )

        self.land_srv = self.create_service(
            HkLand,
            '/hk/land',
            self._land_callback,
        )

        self.emergency_srv = self.create_service(
            Trigger,
            '/hk/emergency',
            self._emergency_callback,
        )

        # Crazyswarm2 command publisher.
        self.full_state_pub = self.create_publisher(
            FullState,
            topic(self.robot_name, 'cmd_full_state'),
            10,
        )

        # Crazyswarm2 service clients.
        self.cf_takeoff = self.create_client(
            CfTakeoff,
            topic(self.robot_name, 'takeoff'),
        )

        self.cf_land = self.create_client(
            CfLand,
            topic(self.robot_name, 'land'),
        )

        self.cf_emergency = self.create_client(
            Empty,
            topic(self.robot_name, 'emergency'),
        )

        self.cf_notify_stop = self.create_client(
            NotifySetpointsStop,
            topic(self.robot_name, 'notify_setpoints_stop'),
        )

        self.get_logger().info(
            f'cf_adapter ready for robot "{self.robot_name}"'
        )

        self.get_logger().info(
            'Project API: /hk/cmd/safe, /hk/takeoff, /hk/land, '
            '/hk/emergency, /hk/adapter/odom, /hk/adapter/ranges'
        )

    def _command_callback(self, msg):
        """Translate a project Setpoint to a Crazyswarm2 FullState command."""
        if msg.mode not in (
            Setpoint.MODE_POSITION,
            Setpoint.MODE_VELOCITY,
            Setpoint.MODE_HOVER,
        ):
            self.get_logger().error(
                f'Rejected setpoint with unsupported mode: {msg.mode}'
            )
            return

        command = FullState()

        command.header = msg.header

        command.pose.position.x = msg.position.x
        command.pose.position.y = msg.position.y
        command.pose.position.z = msg.position.z

        qx, qy, qz, qw = yaw_to_quaternion(msg.yaw)

        command.pose.orientation.x = qx
        command.pose.orientation.y = qy
        command.pose.orientation.z = qz
        command.pose.orientation.w = qw

        if msg.mode == Setpoint.MODE_VELOCITY:
            command.twist.linear.x = msg.velocity.x
            command.twist.linear.y = msg.velocity.y
            command.twist.linear.z = msg.velocity.z
        else:
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.0

        command.twist.angular.x = 0.0
        command.twist.angular.y = 0.0
        command.twist.angular.z = 0.0

        command.acc.x = 0.0
        command.acc.y = 0.0
        command.acc.z = 0.0

        self.full_state_pub.publish(command)

    def _takeoff_callback(self, request, response):
        """Forward the project takeoff request to Crazyswarm2."""
        if request.height <= 0.0:
            response.success = False
            response.message = 'Takeoff height must be positive'
            return response

        if request.duration <= 0.0:
            response.success = False
            response.message = 'Takeoff duration must be positive'
            return response

        if not self.cf_takeoff.service_is_ready():
            response.success = False
            response.message = 'Crazyflie takeoff service is not available'
            return response

        cf_request = CfTakeoff.Request()
        cf_request.group_mask = self.group_mask
        cf_request.height = request.height

        sec, nanosec = seconds_to_duration_parts(request.duration)
        cf_request.duration.sec = sec
        cf_request.duration.nanosec = nanosec

        self._queue_high_level_request(
            self.cf_takeoff,
            cf_request,
            'takeoff',
        )

        response.success = True
        response.message = 'Takeoff request forwarded to Crazyswarm2'

        return response

    def _land_callback(self, request, response):
        """Forward the project land request to Crazyswarm2."""
        if request.duration <= 0.0:
            response.success = False
            response.message = 'Landing duration must be positive'
            return response

        if not self.cf_land.service_is_ready():
            response.success = False
            response.message = 'Crazyflie land service is not available'
            return response

        cf_request = CfLand.Request()
        cf_request.group_mask = self.group_mask
        cf_request.height = self.land_height

        sec, nanosec = seconds_to_duration_parts(request.duration)
        cf_request.duration.sec = sec
        cf_request.duration.nanosec = nanosec

        self._queue_high_level_request(
            self.cf_land,
            cf_request,
            'land',
        )

        response.success = True
        response.message = 'Land request forwarded to Crazyswarm2'

        return response

    def _emergency_callback(self, request, response):
        """Forward an emergency request immediately to Crazyswarm2."""
        del request

        if not self.cf_emergency.service_is_ready():
            response.success = False
            response.message = 'Crazyflie emergency service is not available'
            return response

        future = self.cf_emergency.call_async(Empty.Request())
        future.add_done_callback(
            partial(self._service_done_callback, label='emergency')
        )

        response.success = True
        response.message = 'Emergency request forwarded to Crazyswarm2'

        return response

    def _queue_high_level_request(self, client, request, label):
        """Release streaming priority before a high-level command."""
        if not self.cf_notify_stop.service_is_ready():
            self.get_logger().warn(
                'notify_setpoints_stop is unavailable; '
                f'forwarding {label} directly'
            )
            self._send_request(client, request, label)
            return

        notify_request = NotifySetpointsStop.Request()
        notify_request.group_mask = self.group_mask
        notify_request.remain_valid_millisecs = self.streaming_valid_ms

        future = self.cf_notify_stop.call_async(notify_request)

        future.add_done_callback(
            lambda _: self._send_request(client, request, label)
        )

    def _send_request(self, client, request, label):
        """Send a Crazyswarm2 service request asynchronously."""
        future = client.call_async(request)
        future.add_done_callback(
            partial(self._service_done_callback, label=label)
        )

    def _service_done_callback(self, future, label):
        """Log completion of an asynchronous Crazyswarm2 service call."""
        try:
            future.result()
            self.get_logger().info(
                f'Crazyswarm2 {label} request completed'
            )
        except Exception as exc:
            self.get_logger().error(
                f'Crazyswarm2 {label} request failed: {exc}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = CrazyflieAdapter()

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