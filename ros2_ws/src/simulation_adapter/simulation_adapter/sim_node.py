"""Our own Crazyflie simulator: replaces cf_adapter during development (TP-10, TP-41).

Reason: the Crazyswarm2 `sim` backend does not publish the Multi-ranger ranges.

Subscribes:
  /hk/cmd/safe              hackathon_interfaces/Setpoint   (only safety_supervisor publishes it)
Publishes:
  /hk/adapter/odom          nav_msgs/Odometry               "estimated" pose + velocity
  /hk/adapter/ranges        hackathon_interfaces/MultiRange synthetic ranges with optional noise
  /hk/sim/ground_truth      nav_msgs/Odometry               real pose, for metrics only
  /hk/sim/obstacles         visualization_msgs/MarkerArray  world obstacles for RViz
Services:
  /hk/takeoff               hackathon_interfaces/Takeoff
  /hk/land                  hackathon_interfaces/Land
  /hk/emergency             std_srvs/Trigger
Parameters:
  world_file, scenario_file, sim_rate, state_rate, range_rate, max_speed, max_accel,
  range_noise_std, pose_noise_std, command_watchdog

Implemented: POSITION/VELOCITY/HOVER commands, services, odometry and world/ranges.
Scenario fault injection integrated from the main contribution; full TP-41 campaign pending.
Logic: world.py (obstacles + raycast), dynamics.py (kinematic model), faults.py (injection).
"""
from collections import deque
import math
import random

import rclpy
from rclpy.duration import Duration
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from hackathon_interfaces.msg import Setpoint, MultiRange
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from .world import World
from .sensors import sample_ranges
from .faults import FaultInjector
from hackathon_interfaces.srv import Takeoff, Land
from std_srvs.srv import Trigger

from .dynamics import DroneState, KinematicDrone, MotionCommand


class SimulationAdapter(Node):

    def __init__(self):
        super().__init__('simulation_adapter')
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        state_rate = self.declare_parameter('state_rate', 50.0).value
        sim_rate = self.declare_parameter('sim_rate', 100.0).value
        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.max_accel = self.declare_parameter('max_accel', 2.0).value
        self.response_time = self.declare_parameter('response_time', 1.0).value
        self.command_watchdog = self.declare_parameter('command_watchdog', 0.5).value
        self.position_tolerance = self.declare_parameter('position_tolerance', 0.01).value
        self.settle_speed = self.declare_parameter('settle_speed', 0.02).value
        self.gravity = self.declare_parameter('gravity', 9.81).value
        self.max_yaw_rate = self.declare_parameter('max_yaw_rate', 1.5).value
        self.range_rate = self.declare_parameter('range_rate', 20.0).value
        self.range_min = self.declare_parameter('range_min', 0.02).value
        self.range_max = self.declare_parameter('range_max', 4.0).value
        self.range_noise_std = self.declare_parameter('range_noise_std', 0.0).value
        self.pose_noise_std = self.declare_parameter('pose_noise_std', 0.0).value
        self.rng = random.Random(self.declare_parameter('random_seed', 0).value)
        world_file = self.declare_parameter('world_file', '').value
        self.world = World((-2, -2, 2, 2), 2.0)
        if world_file:
            with open(world_file, encoding='utf-8') as f:
                self.world = World.from_dict(yaml.safe_load(f))
        if not math.isfinite(self.range_rate) or self.range_rate <= 0:
            raise ValueError('range_rate must be positive and finite')
        if not math.isfinite(self.pose_noise_std) or self.pose_noise_std < 0:
            raise ValueError('pose_noise_std must be finite and non-negative')
        scenario_file = self.declare_parameter('scenario_file', '').value
        start = self.declare_parameter('initial_position', [0.0, 0.0, 0.0, 0.0]).value
        self.scenario = {}
        if scenario_file:
            with open(scenario_file, encoding='utf-8') as f:
                self.scenario = yaml.safe_load(f) or {}
                start = self.scenario.get('start', start)
        self.fault_injector = FaultInjector(self.scenario.get('faults', []))
        parameters = (state_rate, sim_rate, self.max_speed, self.max_accel,
                      self.response_time, self.command_watchdog)
        if not all(math.isfinite(value) and value > 0 for value in parameters):
            raise ValueError('Rates, limits and times must be finite and positive')
        if len(start) != 4 or not all(math.isfinite(value) for value in start):
            raise ValueError('The initial position must contain finite x, y, z, yaw')
        self.dt = 1.0 / sim_rate
        if self.dt > self.response_time:
            raise ValueError('The simulation period must be shorter than response_time')
        initial = DroneState(x=float(start[0]), y=float(start[1]), z=float(start[2]),
                             yaw=float(start[3]), flying=start[2] > 0)
        self.drone = KinematicDrone(self.max_speed, self.max_accel, self.command_watchdog,
                                    initial, self.response_time, self.position_tolerance,
                                    self.settle_speed, self.gravity, self.max_yaw_rate)
        self.elapsed = 0.0
        self.pose_history = deque([(0.0, self._pose_dict())])
        sample_ranges(self.world, initial, 0, self.range_min, self.range_max, self.range_noise_std, self.rng)
        self.last_command_stamp_ns = None

        self.truth_pub = self.create_publisher(Odometry, '/hk/sim/ground_truth', 10)
        self.ranges_pub = self.create_publisher(MultiRange, '/hk/adapter/ranges', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/hk/sim/obstacles', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/hk/adapter/odom', 10)
        self.command_sub = self.create_subscription(Setpoint, '/hk/cmd/safe', self._command_callback, 10)
        self.takeoff_srv = self.create_service(Takeoff, '/hk/takeoff', self._takeoff_callback)
        self.land_srv = self.create_service(Land, '/hk/land', self._land_callback)
        self.emergency_srv = self.create_service(Trigger, '/hk/emergency', self._emergency_callback)
        self.sim_timer = self.create_timer(self.dt, self._step)
        self.state_timer = self.create_timer(1.0 / state_rate, self._publish_odom)
        self.range_timer = self.create_timer(1.0 / self.range_rate, self._publish_ranges)
        self.marker_timer = self.create_timer(0.1, self._publish_markers)
        self.get_logger().info('TP-10 simulator ready: commands, services, odometry and six ranges')

    @property
    def state(self):
        return self.drone.state

    @property
    def target(self):
        return self.drone.target

    @property
    def mode(self):
        return self.drone.mode

    @property
    def last_command_time(self):
        return self.drone.last_command_time

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pose_dict(self):
        return {name: getattr(self.state, name) for name in ('x', 'y', 'z', 'yaw')}

    def _command_callback(self, msg):
        if self.fault_injector.drop_command(self.elapsed):
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        stamp = stamp_ns / 1e9
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        values = (msg.position.x, msg.position.y, msg.position.z, msg.yaw,
                  msg.velocity.x, msg.velocity.y, msg.velocity.z)
        if msg.header.frame_id != self.map_frame or not 0 <= age <= self.command_watchdog:
            self.get_logger().warn('Command rejected: wrong frame or stale/future timestamp')
            return
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('Comando rechazado: valores no finitos')
            return
        if self.last_command_stamp_ns is not None and stamp_ns < self.last_command_stamp_ns:
            return
        try:
            self.drone.set_command(stamp, MotionCommand(
                msg.mode, (msg.position.x, msg.position.y, msg.position.z), msg.yaw,
                (msg.velocity.x, msg.velocity.y, msg.velocity.z)))
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        self.last_command_stamp_ns = stamp_ns

    def _step(self):
        previous_phase = self.drone.phase
        previous_command = self.last_command_time
        self.drone.step(self._now(), self.dt)
        self.elapsed += self.dt
        self.pose_history.append((self.elapsed, self._pose_dict()))
        while len(self.pose_history) > 1 and self.elapsed-self.pose_history[0][0] > 10:
            self.pose_history.popleft()
        if previous_command is not None and self.last_command_time is None:
            self.last_command_stamp_ns = None
            self.get_logger().warn('Command timeout: capturing a HOVER reference')
        if previous_phase != self.drone.phase:
            self.get_logger().info(f'Estado simulado: {previous_phase} → {self.drone.phase}')

    def _takeoff_callback(self, request, response):
        try:
            self.drone.takeoff(request.height, request.duration)
            self.last_command_stamp_ns = None
            response.success = True
            response.message = 'Takeoff accepted; check the height in the odometry to confirm arrival'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _land_callback(self, request, response):
        try:
            self.drone.land(request.duration)
            self.last_command_stamp_ns = None
            response.success = True
            response.message = 'Landing accepted; check ground contact in the odometry'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _emergency_callback(self, request, response):
        del request
        self.drone.emergency_stop()
        self.last_command_stamp_ns = None
        response.success = True
        response.message = 'Simulated cut-off: gravity fall; the simulator must be restarted'
        return response

    def _publish_odom(self):
        state = self.state
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.position.z = state.z
        msg.pose.pose.orientation.z = math.sin(state.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(state.yaw / 2.0)
        # Odometry expresses the twist in the body frame, not in map.
        c, s = math.cos(state.yaw), math.sin(state.yaw)
        msg.twist.twist.linear.x = c * state.vx + s * state.vy
        msg.twist.twist.linear.y = -s * state.vx + c * state.vy
        msg.twist.twist.linear.z = state.vz
        msg.twist.twist.angular.z = state.yaw_rate
        self.truth_pub.publish(msg)
        measured = self.fault_injector.apply_to_pose(
            self.elapsed, self._pose_dict(), list(self.pose_history))
        if measured is None:
            return  # Ground truth continues even if the estimate is interrupted.
        for axis in ('x', 'y', 'z'):
            setattr(msg.pose.pose.position, axis, measured[axis])
        msg.pose.pose.orientation.z = math.sin(measured['yaw']/2)
        msg.pose.pose.orientation.w = math.cos(measured['yaw']/2)
        # A delayed or frozen pose keeps the timestamp of its sample, as a late or stalled
        # estimator would; a jumped pose keeps the publication timestamp.
        if 'stamp_t' in measured:
            age = Duration(seconds=max(0.0, self.elapsed - measured['stamp_t']))
            msg.header.stamp = (self.get_clock().now() - age).to_msg()
        # Noise-free real reference above; synthetic noisy estimate below.
        for axis in ('x', 'y', 'z'):
            value = getattr(msg.pose.pose.position, axis)
            setattr(msg.pose.pose.position, axis, value + self.rng.gauss(0, self.pose_noise_std))
        for index in (0, 7, 14):
            msg.pose.covariance[index] = self.pose_noise_std**2
        self.odom_pub.publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(transform)

    def _publish_ranges(self):
        msg = MultiRange()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.range_min, msg.range_max = float(self.range_min), float(self.range_max)
        values = sample_ranges(self.world, self.state, self.elapsed,
                               self.range_min, self.range_max, self.range_noise_std, self.rng)
        values = self.fault_injector.apply_to_ranges(self.elapsed, values)
        for name, value in values.items():
            setattr(msg, name, float(value))
        self.ranges_pub.publish(msg)

    def _publish_markers(self):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        # Room and boxes in map; timing/motion use the time relative to start-up.
        x0, y0, x1, y1 = self.world.bounds
        h = self.world.ceiling
        solids = [((x0, (y0+y1)/2, h/2), (0.02, y1-y0, h)),
                  ((x1, (y0+y1)/2, h/2), (0.02, y1-y0, h)),
                  (((x0+x1)/2, y0, h/2), (x1-x0, 0.02, h)),
                  (((x0+x1)/2, y1, h/2), (x1-x0, 0.02, h)),
                  (((x0+x1)/2, (y0+y1)/2, h), (x1-x0, y1-y0, 0.02)),
                  (((x0+x1)/2, (y0+y1)/2, 0.0), (x1-x0, y1-y0, 0.02))]
        for box in self.world.obstacles:
            if box.active(self.elapsed):
                x, y = box.center_at(self.elapsed)
                height = min(box.height, h)
                solids.append(((x, y, height/2), (*box.size, height)))
        for index, (position, size) in enumerate(solids):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = 'world', index
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, position)
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, size)
            marker.color.r, marker.color.g, marker.color.b = 0.6, 0.4, 0.2
            marker.color.a = 0.25 if index < 6 else 0.8
            array.markers.append(marker)
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationAdapter()
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
