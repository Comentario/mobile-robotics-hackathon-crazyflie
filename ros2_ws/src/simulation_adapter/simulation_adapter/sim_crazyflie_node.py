"""Dron simulado con la MISMA API ROS que crazyflie_server de Crazyswarm2 (TP-10).

Objetivo: que cf_adapter no note si debajo hay un dron simulado o el real.
En Dresden este nodo se sustituye por crazyflie_server + Crazyradio + dron real;
cf_adapter y el resto del stack no cambian.

API copiada de Crazyswarm2 1.0.7 (crazyflie_server_cpp/src/crazyflie_server.cpp):

Suscribe:
  /<robot>/cmd_full_state        crazyflie_interfaces/FullState   setpoint de bajo nivel
Servicios:
  /<robot>/takeoff               crazyflie_interfaces/Takeoff     high-level commander
  /<robot>/land                  crazyflie_interfaces/Land        high-level commander
  /<robot>/emergency             std_srvs/Empty                   motores fuera
  /<robot>/notify_setpoints_stop crazyflie_interfaces/NotifySetpointsStop
Publica:
  /<robot>/pose                  geometry_msgs/PoseStamped  frame 'world' (estado del "EKF")
  TF world -> <robot>            igual que la pose (Crazyswarm2 publica los dos)
  /<robot>/scan                  sensor_msgs/LaserScan      [back, right, front, left]
  /<robot>/ranges_extra          crazyflie_interfaces/LogDataGeneric  [range.up, range.zrange] mm
Solo de simulación (no existen con el dron real):
  /hk/sim/ground_truth           nav_msgs/Odometry          pose real, para métricas
  /hk/sim/obstacles              visualization_msgs/MarkerArray

Qué bloque del dron real imita cada parte (ver docs/sensor_model.md)
-------------------------------------------------------------------
  Commander, controlador, motores -> dynamics.py (KinematicDrone, firmware_watchdog=True)
  EKF (pose estimada)             -> pose real + ruido + fallos (faults.py)
  Multi-ranger, ToF del Flow deck -> world.py (raycast) + ruido + fallos (faults.py)
  Lighthouse deck, bases, IMU     -> no se simulan (su efecto está en el ruido de pose)
  Sala                            -> world.py + config/worlds/*.yaml

Comportamientos del firmware real que se imitan
-----------------------------------------------
  - Prioridad: tras recibir cmd_full_state, takeoff/land se IGNORAN hasta que se
    llama a notify_setpoints_stop (commanders_setpoints de Bitcraze: "Low level
    setpoints will always have higher priority").
  - Watchdog (supervisor.c, firmware 2025.02): sin setpoints de bajo nivel 0.5 s ->
    nivelado (deriva); 2.0 s -> motores fuera (cae).
  - emergency: motores fuera, el dron cae y no acepta más órdenes (hay que
    reiniciar el nodo, como reiniciar el dron real).
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

from .dynamics import Command, KinematicDrone
from .faults import RANGE_SENSORS, FaultInjector
from .world import World

POSE_HISTORY_SECONDS = 5.0

# Rangers (VL53L1x): alcance físico simulado. Crazyswarm2 recorta el scan a 3.49 m.
SENSOR_MAX_RANGE = 4.0
SCAN_RANGE_MIN = 0.01        # crazyflie_server.cpp: msg.range_min = 0.01
SCAN_RANGE_MAX = 3.49        # crazyflie_server.cpp: max_range = 3.49 -> +inf
# Valor en mm que se publica en ranges_extra cuando no hay retorno.
# Nota mía: el valor exacto del firmware real está por verificar en Dresden;
# cf_adapter trata cualquier valor > range_max como +inf.
NO_RETURN_MM = 8000.0

DRONE_RADIUS = 0.06          # m, nota mía: aproximado para el Crazyflie 2.1 Brushless

SENSOR_DIRECTIONS = {
    'front': (1.0, 0.0, 0.0),
    'back': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
}


def quaternion_to_yaw(q):
    """Yaw (rad) de un cuaternión geometry_msgs."""
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

        # Escenario: pose inicial y fallos. Mundo: paredes y obstáculos.
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

        # Dron: dynamics.py con el watchdog del firmware real.
        self.drone = KinematicDrone(max_speed, max_accel, levelout_timeout,
                                    firmware_watchdog=True,
                                    shutdown_timeout=shutdown_timeout)
        self.drone.reset_pose(*(float(v) for v in start))
        self.dt = 1.0 / sim_rate
        self.sim_time = 0.0
        self._last_mode = self.drone.mode
        self.low_level_priority = False      # True tras recibir cmd_full_state
        self.pose_history = deque(maxlen=int(POSE_HISTORY_SECONDS * pose_rate))

        name = lambda suffix: f'/{self.robot}/{suffix}'   # noqa: E731

        # --- API de crazyflie_server ------------------------------------------
        self.create_subscription(FullState, name('cmd_full_state'), self._on_full_state, 10)
        self.create_service(Takeoff, name('takeoff'), self._on_takeoff)
        self.create_service(Land, name('land'), self._on_land)
        self.create_service(Empty, name('emergency'), self._on_emergency)
        self.create_service(NotifySetpointsStop, name('notify_setpoints_stop'), self._on_notify)
        self.pose_pub = self.create_publisher(PoseStamped, name('pose'), 10)
        self.scan_pub = self.create_publisher(LaserScan, name('scan'), 10)
        self.extra_pub = self.create_publisher(LogDataGeneric, name('ranges_extra'), 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Solo simulación ----------------------------------------------------
        self.truth_pub = self.create_publisher(Odometry, '/hk/sim/ground_truth', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/hk/sim/obstacles', 10)

        self.create_timer(self.dt, self._step)
        self.create_timer(1.0 / pose_rate, self._publish_pose)
        self.create_timer(1.0 / scan_rate, self._publish_ranges)
        self.create_timer(1.0, self._publish_obstacles)

        self.get_logger().info(
            f'sim_crazyflie listo: robot {self.robot}, pose inicial {start}, '
            f'mundo {world_file or "vacío"} ({len(self.world.obstacles)} obstáculos), '
            f'fallos {[f["type"] for f in faults] or "ninguno"}'
        )

    # ------------------------------------------------------------------
    # API de crazyflie_server
    # ------------------------------------------------------------------
    def _on_full_state(self, msg):
        """Setpoint de bajo nivel: posición + yaw (la velocidad se ignora)."""
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
                f'cmd_full_state ignorado: el dron está en modo {self.drone.mode}',
                throttle_duration_sec=2.0,
            )

    def _on_takeoff(self, request, response):
        if self.low_level_priority:
            self.get_logger().warn('takeoff ignorado: prioridad de bajo nivel activa '
                                   '(falta notify_setpoints_stop), como en el firmware')
            return response
        ok, message = self.drone.takeoff(request.height, duration_to_sec(request.duration))
        self.get_logger().info(f'takeoff -> {message}')
        return response

    def _on_land(self, request, response):
        if self.low_level_priority:
            self.get_logger().warn('land ignorado: prioridad de bajo nivel activa '
                                   '(falta notify_setpoints_stop), como en el firmware')
            return response
        ok, message = self.drone.land(duration_to_sec(request.duration))
        self.get_logger().info(f'land -> {message}')
        return response

    def _on_emergency(self, request, response):
        del request
        self.drone.emergency_stop('emergency')
        self.get_logger().error('emergency -> motores apagados, el dron cae')
        return response

    def _on_notify(self, request, response):
        """El high-level commander recupera el control (hover donde está)."""
        self.low_level_priority = False
        self.drone.high_level_hover()
        self.get_logger().info(
            f'notify_setpoints_stop (remain_valid {request.remain_valid_millisecs} ms) '
            '-> high-level commander activo'
        )
        return response

    # ------------------------------------------------------------------
    # Simulación
    # ------------------------------------------------------------------
    def _step(self):
        self.sim_time += self.dt
        self.drone.step(self.sim_time, self.dt)
        s = self.drone.state

        if self.drone.mode != self._last_mode:
            extra = ''
            if self.drone.mode == 'level_out':
                extra = ' (watchdog del firmware: 0.5 s sin setpoints, el dron deriva)'
            elif self.drone.mode == 'emergency' and self.drone.emergency_reason == 'watchdog':
                extra = ' (watchdog del firmware: 2 s sin setpoints, MOTORES FUERA)'
            log = self.get_logger().error if self.drone.mode == 'emergency' else \
                self.get_logger().info
            log(f'Modo: {self._last_mode} -> {self.drone.mode}{extra}')
            if self.drone.mode in ('idle', 'hover', 'emergency'):
                self.low_level_priority = False
            self._last_mode = self.drone.mode

        active = self.faults.active_types(self.sim_time)
        if active != self._last_faults:
            self.get_logger().warn(f't={self.sim_time:.1f} s  fallos activos: {active or "ninguno"}')
            self._last_faults = active

        before = self.world.collision_count
        self.world.collides((s.x, s.y, s.z), DRONE_RADIUS, self.sim_time)
        if self.world.collision_count != before:
            self.get_logger().error(
                f'COLISIÓN #{self.world.collision_count} en t={self.sim_time:.1f} s '
                f'pos=({s.x:.2f}, {s.y:.2f}, {s.z:.2f})'
            )

    def _sample(self, stamp_sec, noise_std):
        s = self.drone.state
        return {
            't': self.sim_time, 'stamp': stamp_sec,
            'x': s.x + random.gauss(0.0, noise_std),
            'y': s.y + random.gauss(0.0, noise_std),
            'z': s.z + random.gauss(0.0, noise_std),
            'yaw': s.yaw, 'vx': s.vx, 'vy': s.vy, 'vz': s.vz,
        }

    def _publish_pose(self):
        """Pose del "EKF" (con ruido y fallos) como /<robot>/pose + TF; pose real aparte."""
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
            return                                   # pose_dropout: no se publica nada

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
        """Multi-ranger + ToF del Flow deck: raycast, ruido, fallos, formato Crazyswarm2."""
        s = self.drone.state
        cos_y, sin_y = math.cos(s.yaw), math.sin(s.yaw)
        ranges = {}
        for sensor in RANGE_SENSORS:
            bx, by, bz = SENSOR_DIRECTIONS[sensor]
            direction = (cos_y * bx - sin_y * by, sin_y * bx + cos_y * by, bz)
            d = self.world.raycast((s.x, s.y, s.z), direction, SENSOR_MAX_RANGE, self.sim_time)
            if math.isfinite(d):
                d = max(0.0, d + random.gauss(0.0, self.range_noise_std))
            ranges[sensor] = d

        ranges = self.faults.apply_to_ranges(self.sim_time, ranges)
        if ranges is None:
            return                                   # range_dropout 'all'

        now = self.get_clock().now().to_msg()

        # /<robot>/scan igual que crazyflie_server.cpp (on_logging_scan).
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

        # /<robot>/ranges_extra: [range.up, range.zrange] en mm (custom topic).
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
