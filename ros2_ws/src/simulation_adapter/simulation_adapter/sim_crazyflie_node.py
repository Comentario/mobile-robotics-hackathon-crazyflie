"""Simulated drone with the SAME ROS API as the Crazyswarm2 crazyflie_server (TP-10).

Goal: cf_adapter must not notice whether a simulated or the real drone is below it.
In Dresden this node is replaced by crazyflie_server + Crazyradio + real drone;
cf_adapter and the rest of the stack do not change.

API copied from Crazyswarm2 1.0.7 (crazyflie_server_cpp/src/crazyflie_server.cpp):

Subscribes:
  /<robot>/cmd_full_state        crazyflie_interfaces/FullState   low-level setpoint
Services:
  /<robot>/takeoff               crazyflie_interfaces/Takeoff     high-level commander
  /<robot>/land                  crazyflie_interfaces/Land        high-level commander
  /<robot>/emergency             std_srvs/Empty                   motors off
  /<robot>/notify_setpoints_stop crazyflie_interfaces/NotifySetpointsStop
Publishes:
  /<robot>/pose                  geometry_msgs/PoseStamped  frame 'world' (synthetic estimate)
  TF world -> <robot>            same as the pose (Crazyswarm2 publishes both)
  /<robot>/scan                  sensor_msgs/LaserScan      [back, right, front, left]
  /<robot>/ranges_extra          crazyflie_interfaces/LogDataGeneric  [range.up, range.zrange] mm
Simulation only (they do not exist with the real drone):
  /hk/sim/ground_truth           nav_msgs/Odometry          real pose, for metrics
  /hk/sim/obstacles              visualization_msgs/MarkerArray

Which block of the real drone each part mimics (see docs/sensor_model.md)
-------------------------------------------------------------------------
  Commander, controller, motors   -> dynamics.py (KinematicDrone, firmware_watchdog=True)
  EKF (estimated pose)            -> real pose + noise + faults (faults.py)
  Multi-ranger, Flow deck ToF     -> world.py (raycast) + noise + faults (faults.py)
  Lighthouse deck, bases, IMU     -> not simulated (their effect is in the pose noise)
  Room                            -> world.py + config/worlds/*.yaml

Firmware behaviours that are approximated, without certifying equivalence
-------------------------------------------------------------------------
  - Priority: after cmd_full_state is received, takeoff/land are IGNORED until
    notify_setpoints_stop is called (Bitcraze commanders_setpoints: "Low level
    setpoints will always have higher priority").
  - Watchdog (supervisor.c, firmware 2025.02): no low-level setpoints for 0.5 s ->
    level out (drift); 2.0 s -> motors off (falls).
  - emergency: motors off, the drone falls and accepts no more commands (the node
    must be restarted, like restarting the real drone).
"""
from collections import deque
import math
import random

import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from crazyflie_interfaces.msg import FullState, LogDataGeneric
from crazyflie_interfaces.srv import Land, NotifySetpointsStop, Takeoff

from .emulator.dynamics import Command, KinematicDrone
from .emulator.faults import RANGE_SENSORS, FaultInjector
from .emulator.world import World

POSE_HISTORY_SECONDS = 5.0

# Rangers (VL53L1x): simulated physical range. Crazyswarm2 clips the scan at 3.49 m.
SENSOR_MAX_RANGE = 4.0
SCAN_RANGE_MIN = 0.01        # crazyflie_server.cpp: msg.range_min = 0.01
SCAN_RANGE_MAX = 3.49        # crazyflie_server.cpp: max_range = 3.49 -> +inf
# Value in mm published in ranges_extra when there is no return.
# Note: the exact value of the real firmware was still to be verified in Dresden;
# cf_adapter treats any value > range_max as +inf.
NO_RETURN_MM = 8000.0

DRONE_RADIUS = 0.06          # m, approximation for the Crazyflie 2.1 Brushless

SENSOR_DIRECTIONS = {
    'front': (1.0, 0.0, 0.0),
    'back': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
}


def quaternion_to_yaw(q):
    """Yaw (rad) of a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def duration_to_sec(duration):
    return duration.sec + duration.nanosec * 1e-9


class SimCrazyflie(Node):

    def __init__(self):
        super().__init__('sim_crazyflie')

        p = self.declare_parameter
        self.robot = p('robot_name', 'cf231').value
        self.world_frame = p('world_frame', 'world').value
        self.map_frame = p('map_frame', 'map').value
        sim_rate = p('sim_rate', 100.0).value
        pose_rate = p('pose_rate', 50.0).value
        scan_rate = p('scan_rate', 20.0).value
        max_speed = p('max_speed', 1.0).value
        max_accel = p('max_accel', 2.0).value
        levelout_timeout = p('levelout_timeout', 0.5).value
        shutdown_timeout = p('shutdown_timeout', 2.0).value
        self.pose_noise_std = p('pose_noise_std', 0.005).value
        self.range_noise_std = p('range_noise_std', 0.01).value
        scenario_file = p('scenario_file', '').value
        world_file = p('world_file', '').value

        # Scenario: initial pose and faults. World: walls and obstacles.
        start, faults = [0.0, 0.0, 0.0, 0.0], []
        if scenario_file:
            with open(scenario_file, encoding='utf-8') as f:
                scenario = yaml.safe_load(f) or {}
            start = scenario.get('start', start)
            faults = scenario.get('faults', []) or []
        self.faults = FaultInjector(faults)
        self._last_faults = []
        if world_file:
            with open(world_file, encoding='utf-8') as f:
                self.world = World.from_dict(yaml.safe_load(f))
        else:
            self.world = World.empty()

        # Drone: dynamics.py with the real firmware watchdog.
        self.drone = KinematicDrone(max_speed, max_accel, levelout_timeout,
                                    firmware_watchdog=True,
                                    shutdown_timeout=shutdown_timeout)
        self.drone.reset_pose(*(float(v) for v in start))
        self.dt = 1.0 / sim_rate
        self.sim_time = 0.0
        self.last_ros_time = self.get_clock().now().nanoseconds / 1e9
        self.rng = random.Random(p('random_seed', 0).value)
        self._last_mode = self.drone.mode
        self.low_level_priority = False      # True after cmd_full_state is received
        self.pose_history = deque(maxlen=int(POSE_HISTORY_SECONDS * pose_rate))

        name = lambda suffix: f'/{self.robot}/{suffix}'   # noqa: E731

        # --- crazyflie_server API ---------------------------------------------
        self.create_subscription(FullState, name('cmd_full_state'), self._on_full_state, 10)
        self.create_service(Takeoff, name('takeoff'), self._on_takeoff)
        self.create_service(Land, name('land'), self._on_land)
        self.create_service(Empty, name('emergency'), self._on_emergency)
        self.create_service(NotifySetpointsStop, name('notify_setpoints_stop'), self._on_notify)
        self.pose_pub = self.create_publisher(PoseStamped, name('pose'), 10)
        self.scan_pub = self.create_publisher(LaserScan, name('scan'), 10)
        self.extra_pub = self.create_publisher(LogDataGeneric, name('ranges_extra'), 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Simulation only ----------------------------------------------------
        self.truth_pub = self.create_publisher(Odometry, '/hk/sim/ground_truth', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/hk/sim/obstacles', 10)

        self.create_timer(self.dt, self._step)
        self.create_timer(1.0 / pose_rate, self._publish_pose)
        self.create_timer(1.0 / scan_rate, self._publish_ranges)
        self.create_timer(1.0, self._publish_obstacles)

        self.get_logger().info(
            f'sim_crazyflie ready: robot {self.robot}, initial pose {start}, '
            f'world {world_file or "empty"} ({len(self.world.obstacles)} obstacles), '
            f'faults {[f["type"] for f in faults] or "none"}'
        )

    # ------------------------------------------------------------------
    # crazyflie_server API
    # ------------------------------------------------------------------
    def _on_full_state(self, msg):
        """Low-level setpoint: position + yaw (the velocity is ignored)."""
        if self.faults.drop_command(self.sim_time):
            return                                   # command_pause
        command = Command(
            mode='position',
            x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z,
            yaw=quaternion_to_yaw(msg.pose.orientation),
        )
        if self.drone.set_command(self.sim_time, command):
            self.low_level_priority = True
        else:
            self.get_logger().warn(
                f'cmd_full_state ignored: the drone is in mode {self.drone.mode}',
                throttle_duration_sec=2.0,
            )

    def _on_takeoff(self, request, response):
        if self.low_level_priority:
            self.get_logger().warn('takeoff ignored: low-level priority active '
                                   '(notify_setpoints_stop missing), as in the firmware')
            return response
        ok, message = self.drone.takeoff(request.height, duration_to_sec(request.duration))
        self.get_logger().info(f'takeoff -> {message}')
        return response

    def _on_land(self, request, response):
        if self.low_level_priority:
            self.get_logger().warn('land ignored: low-level priority active '
                                   '(notify_setpoints_stop missing), as in the firmware')
            return response
        ok, message = self.drone.land(duration_to_sec(request.duration))
        self.get_logger().info(f'land -> {message}')
        return response

    def _on_emergency(self, request, response):
        del request
        self.drone.emergency_stop('emergency')
        self.get_logger().error('emergency -> motors off, the drone falls')
        return response

    def _on_notify(self, request, response):
        """The high-level commander takes control back (hover where the drone is)."""
        self.low_level_priority = False
        self.drone.high_level_hover()
        self.get_logger().info(
            f'notify_setpoints_stop (remain_valid {request.remain_valid_millisecs} ms) '
            '-> high-level commander active'
        )
        return response

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------
    def _step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_ros_time
        self.last_ros_time = now
        if dt <= 0:
            return
        # Substeps keep the integration stable when a callback arrives late.
        remaining = min(dt, 0.5)
        while remaining > 1e-9:
            step = min(self.dt, remaining)
            self.sim_time += step
            self.drone.step(self.sim_time, step)
            remaining -= step
        s = self.drone.state

        if self.drone.mode != self._last_mode:
            extra = ''
            if self.drone.mode == 'level_out':
                extra = ' (firmware watchdog: 0.5 s without setpoints, the drone drifts)'
            elif self.drone.mode == 'emergency' and self.drone.emergency_reason == 'watchdog':
                extra = ' (firmware watchdog: 2 s without setpoints, MOTORS OFF)'
            log = self.get_logger().error if self.drone.mode == 'emergency' else \
                self.get_logger().info
            log(f'Mode: {self._last_mode} -> {self.drone.mode}{extra}')
            if self.drone.mode in ('idle', 'hover', 'emergency'):
                self.low_level_priority = False
            self._last_mode = self.drone.mode

        active = self.faults.active_types(self.sim_time)
        if active != self._last_faults:
            self.get_logger().warn(f't={self.sim_time:.1f} s  active faults: {active or "none"}')
            self._last_faults = active

        before = self.world.collision_count
        self.world.collides((s.x, s.y, s.z), DRONE_RADIUS, self.sim_time)
        if self.world.collision_count != before:
            self.get_logger().error(
                f'COLLISION #{self.world.collision_count} at t={self.sim_time:.1f} s '
                f'pos=({s.x:.2f}, {s.y:.2f}, {s.z:.2f})'
            )

    def _sample(self, stamp_sec, noise_std):
        s = self.drone.state
        return {
            't': self.sim_time, 'stamp': stamp_sec,
            'x': s.x + self.rng.gauss(0.0, noise_std),
            'y': s.y + self.rng.gauss(0.0, noise_std),
            'z': s.z + self.rng.gauss(0.0, noise_std),
            'yaw': s.yaw, 'vx': s.vx, 'vy': s.vy, 'vz': s.vz,
        }

    def _publish_pose(self):
        """'EKF' pose (with noise and faults) as /<robot>/pose + TF; real pose separately."""
        stamp_sec = self.get_clock().now().nanoseconds * 1e-9

        truth = self._sample(stamp_sec, 0.0)
        odom = Odometry()
        odom.header.stamp = self._stamp(truth['stamp'])
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = truth['x']
        odom.pose.pose.position.y = truth['y']
        odom.pose.pose.position.z = truth['z']
        odom.pose.pose.orientation.z = math.sin(truth['yaw'] / 2.0)
        odom.pose.pose.orientation.w = math.cos(truth['yaw'] / 2.0)
        odom.twist.twist.linear.x = truth['vx']
        odom.twist.twist.linear.y = truth['vy']
        odom.twist.twist.linear.z = truth['vz']
        self.truth_pub.publish(odom)

        sample = self._sample(stamp_sec, self.pose_noise_std)
        self.pose_history.append(sample)
        pose = self.faults.apply_to_pose(self.sim_time, sample, self.pose_history)
        if pose is None:
            return                                   # pose_dropout: nothing is published

        msg = PoseStamped()
        msg.header.stamp = self._stamp(pose['stamp'])
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = pose['x']
        msg.pose.position.y = pose['y']
        msg.pose.position.z = pose['z']
        msg.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
        msg.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
        self.pose_pub.publish(msg)

        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self.robot
        tf.transform.translation.x = pose['x']
        tf.transform.translation.y = pose['y']
        tf.transform.translation.z = pose['z']
        tf.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def _publish_ranges(self):
        """Multi-ranger + Flow deck ToF: raycast, noise, faults, Crazyswarm2 format."""
        s = self.drone.state
        cos_y, sin_y = math.cos(s.yaw), math.sin(s.yaw)
        ranges = {}
        for sensor in RANGE_SENSORS:
            bx, by, bz = SENSOR_DIRECTIONS[sensor]
            direction = (cos_y * bx - sin_y * by, sin_y * bx + cos_y * by, bz)
            d = self.world.raycast((s.x, s.y, s.z), direction, SENSOR_MAX_RANGE, self.sim_time)
            if math.isfinite(d):
                d = max(0.0, d + self.rng.gauss(0.0, self.range_noise_std))
            ranges[sensor] = d

        ranges = self.faults.apply_to_ranges(self.sim_time, ranges)
        if ranges is None:
            return                                   # range_dropout 'all'

        now = self.get_clock().now().to_msg()

        # /<robot>/scan same as crazyflie_server.cpp (on_logging_scan).
        def scan_value(d):
            return math.inf if d > SCAN_RANGE_MAX else d
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self.robot
        scan.range_min = SCAN_RANGE_MIN
        scan.range_max = SCAN_RANGE_MAX
        scan.angle_min = -math.pi
        scan.angle_max = 0.5 * math.pi
        scan.angle_increment = math.pi / 2.0
        scan.ranges = [float(scan_value(ranges[k])) for k in ('back', 'right', 'front', 'left')]
        self.scan_pub.publish(scan)

        # /<robot>/ranges_extra: [range.up, range.zrange] in mm (custom topic).
        def mm(d):
            if math.isnan(d):
                return math.nan
            return NO_RETURN_MM if math.isinf(d) else d * 1000.0
        extra = LogDataGeneric()
        extra.header.stamp = now
        extra.header.frame_id = self.world_frame
        extra.timestamp = int(self.sim_time * 1000.0)
        extra.values = [float(mm(ranges['up'])), float(mm(ranges['down']))]
        self.extra_pub.publish(extra)

    def _publish_obstacles(self):
        markers = MarkerArray()
        for i, box in enumerate(self.world.obstacles):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type = 'obstacles', i, Marker.CUBE
            if not box.active(self.sim_time):
                m.action = Marker.DELETE
                markers.markers.append(m)
                continue
            (x0, y0, _), (x1, y1, _) = box.bounds_at(self.sim_time)
            height = box.height if math.isfinite(box.height) else self.world.ceiling
            height = height if math.isfinite(height) else 2.0
            m.pose.position.x, m.pose.position.y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            m.pose.position.z = height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = x1 - x0, y1 - y0, height
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.3, 0.2, 0.6
            markers.markers.append(m)
        self.obstacles_pub.publish(markers)

    @staticmethod
    def _stamp(sec_float):
        t = Time()
        t.sec = int(sec_float)
        t.nanosec = int((sec_float - t.sec) * 1e9)
        return t


def main(args=None):
    rclpy.init(args=args)
    node = SimCrazyflie()
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
