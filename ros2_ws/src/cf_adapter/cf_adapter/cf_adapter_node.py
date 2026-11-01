"""Translate /hk to Crazyswarm2. Parameters/limitations: docs/interfaces.md.

source_frame and map_frame must be aligned. Services acknowledge remote requests,
not completed maneuvers. VELOCITY integrates XY references; HOVER latches pose.
"""
import copy
import math
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger
from tf2_msgs.msg import TFMessage
from crazyflie_interfaces.msg import FullState, LogDataGeneric
from crazyflie_interfaces.srv import Land as CfLand, NotifySetpointsStop, Takeoff as CfTakeoff
from hackathon_interfaces.msg import MultiRange, Setpoint
from hackathon_interfaces.srv import Land as HkLand, Takeoff as HkTakeoff
from .mapping import ReferenceTranslator, scan_to_ranges, seconds_to_duration_parts, topic, yaw_to_quaternion, twist_transform, transform_twist


class CrazyflieAdapter(Node):
    def __init__(self):
        super().__init__('cf_adapter')
        defaults = {
            'robot_name': 'cf231', 'group_mask': 0, 'land_height': 0.0,
            'streaming_valid_ms': 100, 'map_frame': 'map', 'base_frame': 'base_link',
            'source_frame': 'world', 'state_source': 'odom', 'odom_topic': '',
            'pose_topic': '', 'scan_topic': '', 'vertical_ranges_topic': '',
            'flow_only': False, 'flow_range_min': 0.01, 'flow_range_max': 4.0,
            'state_timeout': 0.3, 'range_timeout': 0.3, 'command_timeout': 0.5,
            'service_timeout': 2.0, 'reference_initial_dt': 0.05, 'reference_max_dt': 0.1,
            'odom_linear_frame': 'body', 'odom_angular_units': 'radians',
        }
        for name, value in defaults.items():
            setattr(self, name, self.declare_parameter(name, value).value)
        if self.state_source not in ('odom', 'pose', 'tf'):
            raise ValueError('state_source must be odom, pose or tf')
        if not self.robot_name.strip('/') or not self.source_frame or not self.map_frame:
            raise ValueError('Robot and frame names must be nonempty')
        if not 0 <= self.group_mask <= 255 or not 0 <= self.streaming_valid_ms < 2**32:
            raise ValueError('Invalid group mask or streaming validity')
        for name in ('state_timeout', 'range_timeout', 'command_timeout', 'service_timeout'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(self.land_height) or self.land_height < 0:
            raise ValueError('Invalid landing height')
        if not (math.isfinite(self.flow_range_min) and math.isfinite(self.flow_range_max)
                and 0 <= self.flow_range_min < self.flow_range_max):
            raise ValueError('Invalid Flow range limits')
        if self.flow_only and not self.vertical_ranges_topic:
            raise ValueError('flow_only requires vertical_ranges_topic')
        self.reference = ReferenceTranslator(self.reference_initial_dt, self.reference_max_dt)
        twist_transform((0, 0, 0, 1), self.odom_linear_frame, self.odom_angular_units)
        self.latest_state = None
        self.stream_active = False
        self.streaming_allowed = True
        self.high_level_busy = False
        self.high_level_until = 0.0
        self.operation = 0
        self.vertical = None
        self.odom_pub = self.create_publisher(Odometry, '/hk/adapter/odom', 10)
        self.ranges_pub = self.create_publisher(MultiRange, '/hk/adapter/ranges', 10)
        self.full_state_pub = self.create_publisher(FullState, topic(self.robot_name, 'cmd_full_state'), 10)
        self.command_sub = self.create_subscription(Setpoint, '/hk/cmd/safe', self._command_callback, 10)
        if self.state_source == 'odom':
            self.state_sub = self.create_subscription(Odometry, self.odom_topic or topic(self.robot_name, 'odom'), self._odom_callback, qos_profile_sensor_data)
        elif self.state_source == 'pose':
            self.state_sub = self.create_subscription(PoseStamped, self.pose_topic or topic(self.robot_name, 'pose'), self._pose_callback, qos_profile_sensor_data)
        else:
            self.state_sub = self.create_subscription(TFMessage, '/tf', self._tf_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic or topic(self.robot_name, 'scan'), self._scan_callback, qos_profile_sensor_data)
        if self.vertical_ranges_topic:
            self.vertical_sub = self.create_subscription(LogDataGeneric, self.vertical_ranges_topic, self._vertical_callback, qos_profile_sensor_data)
        self.service_group = ReentrantCallbackGroup()
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.cf_takeoff = self.create_client(CfTakeoff, topic(self.robot_name, 'takeoff'), callback_group=self.service_group)
        self.cf_land = self.create_client(CfLand, topic(self.robot_name, 'land'), callback_group=self.service_group)
        self.cf_emergency = self.create_client(Empty, topic(self.robot_name, 'emergency'), callback_group=self.service_group)
        self.cf_notify_stop = self.create_client(NotifySetpointsStop, topic(self.robot_name, 'notify_setpoints_stop'), callback_group=self.service_group)
        self.takeoff_srv = self.create_service(HkTakeoff, '/hk/takeoff', self._takeoff_callback, callback_group=self.service_group)
        self.land_srv = self.create_service(HkLand, '/hk/land', self._land_callback, callback_group=self.service_group)
        self.emergency_srv = self.create_service(Trigger, '/hk/emergency', self._emergency_callback, callback_group=self.service_group)
        self.get_logger().info(f'Adapter ready: {self.robot_name}, state_source={self.state_source}; {self.source_frame} aligned with {self.map_frame}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _age(self, stamp):
        return self._now() - (stamp.sec + stamp.nanosec / 1e9)

    def _fresh(self, stamp, timeout):
        return 0 <= self._age(stamp) <= timeout

    def _odom_callback(self, msg):
        if msg.header.frame_id not in (self.source_frame, self.map_frame):
            return
        if msg.child_frame_id not in ('', self.robot_name.strip('/'), self.base_frame):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = sum(value * value for value in (q.x, q.y, q.z, q.w))
        if not all(map(math.isfinite, values)) or abs(norm - 1.0) > 0.01:
            self.get_logger().warn('Rejected invalid pose or quaternion')
            return
        if not self._fresh(msg.header.stamp, self.state_timeout):
            return
        if self.latest_state is not None and self._age(msg.header.stamp) > self._age(self.latest_state.header.stamp):
            return
        state = copy.deepcopy(msg)
        state.header.frame_id = self.map_frame
        state.child_frame_id = self.base_frame
        if self.state_source == 'odom':
            linear, angular = state.twist.twist.linear, state.twist.twist.angular
            try:
                values, covariance = transform_twist(
                    (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z),
                    state.twist.covariance,
                    twist_transform((q.x, q.y, q.z, q.w), self.odom_linear_frame, self.odom_angular_units))
            except ValueError:
                return
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z = values
            state.twist.covariance = covariance
        self.latest_state = state
        self.odom_pub.publish(state)

    def _pose_callback(self, msg):
        state = Odometry()
        state.header = copy.deepcopy(msg.header)
        state.child_frame_id = self.robot_name.strip('/')
        state.pose.pose = copy.deepcopy(msg.pose)
        for index in (0, 7, 14, 21, 28, 35):
            state.twist.covariance[index] = 1e6
        self._odom_callback(state)

    def _tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == self.source_frame and transform.child_frame_id == self.robot_name.strip('/'):
                pose = PoseStamped()
                pose.header = copy.deepcopy(transform.header)
                t = transform.transform.translation
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
                pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
                self._pose_callback(pose)

    def _vertical_callback(self, msg):
        if self.flow_only:
            if len(msg.values) != 1 or not self._fresh(msg.header.stamp, self.range_timeout):
                return
            output = MultiRange()
            output.header = copy.deepcopy(msg.header)
            output.header.frame_id = self.base_frame
            output.range_min, output.range_max = self.flow_range_min, self.flow_range_max
            for name in ('front', 'back', 'left', 'right', 'up', 'down'):
                setattr(output, name, float('nan'))
            value = float(msg.values[0]) / 1000.0
            output.down = (float('inf') if value > self.flow_range_max else
                           float('-inf') if value < self.flow_range_min else value)
            self.ranges_pub.publish(output)
            return
        # Custom logging vars must be exactly [range.up, range.zrange], in mm.
        if len(msg.values) != 2 or not self._fresh(msg.header.stamp, self.range_timeout):
            return
        self.vertical = (copy.deepcopy(msg.header.stamp), tuple(float(value) / 1000 for value in msg.values))

    def _scan_callback(self, msg):
        if self.flow_only:
            return
        if msg.header.frame_id not in (self.robot_name.strip('/'), self.base_frame):
            return
        if not self._fresh(msg.header.stamp, self.range_timeout):
            return
        if not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max) or not 0 <= msg.range_min < msg.range_max:
            return
        try:
            rays = scan_to_ranges(msg.ranges, msg.angle_min, msg.angle_increment)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        output = MultiRange()
        output.header = copy.deepcopy(msg.header)
        output.header.frame_id = self.base_frame
        output.range_min, output.range_max = msg.range_min, msg.range_max
        for name, value in rays.items():
            setattr(output, name, value)
        output.up = output.down = float('nan')
        if self.vertical is not None and self._fresh(self.vertical[0], self.range_timeout):
            if self._age(self.vertical[0]) > self._age(output.header.stamp):
                output.header.stamp = copy.deepcopy(self.vertical[0])
            output.up, output.down = self.vertical[1]
            for name in ('up', 'down'):
                value = getattr(output, name)
                if value > output.range_max:
                    setattr(output, name, float('inf'))
                elif value < output.range_min:
                    setattr(output, name, float('-inf'))
        self.ranges_pub.publish(output)

    def _command_callback(self, msg):
        if not self.streaming_allowed or self.high_level_busy or self._now() < self.high_level_until:
            return
        if msg.header.frame_id != self.map_frame or not self._fresh(msg.header.stamp, self.command_timeout):
            self.reference.reset()
            self.get_logger().warn('Rejected stale setpoint or unsupported frame')
            return
        state = self.latest_state
        if state is None or not self._fresh(state.header.stamp, self.state_timeout):
            self.reference.reset()
            self.get_logger().warn('Rejected setpoint: no fresh state')
            return
        p, q = state.pose.pose.position, state.pose.pose.orientation
        measured_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        try:
            position, velocity, yaw = self.reference.translate(
                msg.mode, (msg.position.x, msg.position.y, msg.position.z),
                (msg.velocity.x, msg.velocity.y, msg.velocity.z), msg.yaw,
                (p.x, p.y, p.z), measured_yaw, self._now())
        except ValueError as exc:
            self.reference.reset()
            self.get_logger().warn(f'Rejected setpoint: {exc}')
            return
        command = FullState()
        command.header = copy.deepcopy(msg.header)
        command.header.frame_id = self.source_frame
        command.pose.position.x, command.pose.position.y, command.pose.position.z = position
        q = yaw_to_quaternion(yaw)
        command.pose.orientation.x, command.pose.orientation.y, command.pose.orientation.z, command.pose.orientation.w = q
        command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = velocity
        self.full_state_pub.publish(command)
        self.stream_active = True

    async def _call(self, client, request):
        if not client.service_is_ready():
            raise RuntimeError(f'{client.srv_name} is unavailable')
        future = client.call_async(request)
        def expire():
            if not future.done() and not future.cancelled():
                client.remove_pending_request(future)
                future.set_exception(RuntimeError(f'{client.srv_name} timed out'))
        timer = self.create_timer(self.service_timeout, expire, callback_group=self.service_group, clock=self.wall_clock)
        try:
            result = await future
            if future.cancelled() or result is None:
                raise RuntimeError(f'{client.srv_name} timed out')
            return result
        finally:
            timer.cancel()
            self.destroy_timer(timer)
            client.remove_pending_request(future)

    async def _high_level(self, client, request, response, label, duration):
        if self.high_level_busy:
            response.success, response.message = False, 'Another high-level request is pending'
            return response
        self.high_level_busy = True
        self.streaming_allowed = False
        self.reference.reset()
        self.operation += 1
        operation = self.operation
        try:
            if not client.service_is_ready():
                raise RuntimeError(f'{client.srv_name} is unavailable')
            if self.stream_active:
                stop = NotifySetpointsStop.Request()
                stop.group_mask, stop.remain_valid_millisecs = self.group_mask, self.streaming_valid_ms
                await self._call(self.cf_notify_stop, stop)
                if operation != self.operation:
                    raise RuntimeError('Request preempted by emergency')
            await self._call(client, request)
            if operation != self.operation:
                raise RuntimeError('Request preempted by emergency')
            self.stream_active = False
            self.streaming_allowed = label == 'takeoff'
            self.high_level_until = self._now() + duration
            response.success = True
            response.message = f'Crazyswarm2 acknowledged {label}; maneuver completion is not confirmed'
        except Exception as exc:
            response.success, response.message = False, str(exc)
            self.get_logger().error(str(exc))
        finally:
            self.high_level_busy = False
        return response

    async def _takeoff_callback(self, request, response):
        if not math.isfinite(request.height) or request.height <= 0:
            response.success, response.message = False, 'Height must be finite and positive'
            return response
        return await self._flight_request(request, response, 'takeoff')

    async def _land_callback(self, request, response):
        return await self._flight_request(request, response, 'land')

    async def _flight_request(self, request, response, label):
        try:
            sec, nanosec = seconds_to_duration_parts(request.duration)
            if request.duration <= 0:
                raise ValueError('Duration must be positive')
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            return response
        cf_request = CfTakeoff.Request() if label == 'takeoff' else CfLand.Request()
        cf_request.group_mask = self.group_mask
        cf_request.height = request.height if label == 'takeoff' else self.land_height
        cf_request.duration.sec, cf_request.duration.nanosec = sec, nanosec
        client = self.cf_takeoff if label == 'takeoff' else self.cf_land
        return await self._high_level(client, cf_request, response, label, request.duration)

    async def _emergency_callback(self, request, response):
        del request
        self.operation += 1
        self.streaming_allowed = False
        self.reference.reset()
        try:
            await self._call(self.cf_emergency, Empty.Request())
            response.success = True
            response.message = 'Crazyswarm2 acknowledged emergency; physical stop is not confirmed'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response


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
