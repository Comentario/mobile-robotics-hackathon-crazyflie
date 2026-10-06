"""Simulador propio del Crazyflie: sustituye a cf_adapter durante el desarrollo (TP-10, TP-41).

Motivo: el backend `sim` de Crazyswarm2 no publica los rangos del Multi-ranger.

Suscribe:
  /hk/cmd/safe              hackathon_interfaces/Setpoint   (solo lo publica safety_supervisor)
Publica:
  /hk/adapter/odom          nav_msgs/Odometry               pose + velocidad "estimadas"
  /hk/adapter/ranges        hackathon_interfaces/MultiRange rangos con ruido y fallos inyectados
  /hk/sim/ground_truth      nav_msgs/Odometry               pose real, solo para métricas
  /hk/sim/obstacles         visualization_msgs/MarkerArray  obstáculos del mundo para RViz
Servicios:
  /hk/takeoff               hackathon_interfaces/Takeoff
  /hk/land                  hackathon_interfaces/Land
  /hk/emergency             std_srvs/Trigger
Parámetros:
  world_file, scenario_file, sim_rate, state_rate, range_rate, max_speed, max_accel,
  range_noise_std, pose_noise_std, command_watchdog

Lógica: world.py (obstáculos + raycast), dynamics.py (modelo cinemático), faults.py (inyección).

Estado de implementación (TP-10 por incrementos)
------------------------------------------------
  Incremento 1 (este):  dron que se mueve. /hk/cmd/safe, /hk/takeoff, /hk/land,
                        /hk/emergency, /hk/adapter/odom (con ruido) y
                        /hk/sim/ground_truth (sin ruido).
  Incremento 2:         fallos de pose (faults.py).
  Incremento 3:         límites de altura / techo.
  Incremento 4:         obstáculos y rangos (world.py): /hk/adapter/ranges y
                        /hk/sim/obstacles.
"""
import math
import random

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from hackathon_interfaces.msg import Setpoint
from hackathon_interfaces.srv import Land, Takeoff

from .dynamics import Command, KinematicDrone


class SimulationAdapter(Node):

    def __init__(self):
        super().__init__('simulation_adapter')

        # --- Parámetros (valores por defecto = simulation.yaml) ---------------
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        sim_rate = self.declare_parameter('sim_rate', 100.0).value
        state_rate = self.declare_parameter('state_rate', 50.0).value
        max_speed = self.declare_parameter('max_speed', 1.0).value
        max_accel = self.declare_parameter('max_accel', 2.0).value
        command_watchdog = self.declare_parameter('command_watchdog', 0.5).value
        self.pose_noise_std = self.declare_parameter('pose_noise_std', 0.005).value
        scenario_file = self.declare_parameter('scenario_file', '').value

        # Pose inicial: la 'start' del escenario, o el origen si no hay escenario.
        start = [0.0, 0.0, 0.0, 0.0]   # x, y, z, yaw
        if scenario_file:
            with open(scenario_file, encoding='utf-8') as f:
                start = (yaml.safe_load(f) or {}).get('start', start)

        # --- Modelo del dron (lógica pura, dynamics.py) ----------------------
        self.drone = KinematicDrone(max_speed, max_accel, command_watchdog)
        self.drone.reset_pose(*(float(v) for v in start))

        # Tiempo de simulación propio: avanza exactamente dt en cada paso.
        # Así la simulación es determinista y no depende de la carga del PC.
        self.dt = 1.0 / sim_rate
        self.sim_time = 0.0
        self._last_mode = self.drone.mode

        # Traducción de los modos del mensaje ROS a los de dynamics.py.
        self._mode_names = {
            Setpoint.MODE_POSITION: 'position',
            Setpoint.MODE_VELOCITY: 'velocity',
            Setpoint.MODE_HOVER: 'hover',
        }

        # --- Publishers --------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '/hk/adapter/odom', 10)
        self.truth_pub = self.create_publisher(Odometry, '/hk/sim/ground_truth', 10)

        # --- Subscriber: comandos del stack -----------------------------------
        self.create_subscription(Setpoint, '/hk/cmd/safe', self._on_setpoint, 10)

        # --- Servicios: misma API que cf_adapter ------------------------------
        self.create_service(Takeoff, '/hk/takeoff', self._on_takeoff)
        self.create_service(Land, '/hk/land', self._on_land)
        self.create_service(Trigger, '/hk/emergency', self._on_emergency)

        # --- Timers -------------------------------------------------------------
        # Uno avanza la física (sim_rate) y otro publica la pose (state_rate).
        self.create_timer(self.dt, self._step)
        self.create_timer(1.0 / state_rate, self._publish_state)

        self.get_logger().info(
            f'simulation_adapter listo (incremento 1). Pose inicial: {start}'
        )

    # ------------------------------------------------------------------
    # Callbacks de entrada
    # ------------------------------------------------------------------
    def _on_setpoint(self, msg):
        """Traduce el Setpoint ROS a un Command y se lo pasa al modelo."""
        mode = self._mode_names.get(msg.mode)
        if mode is None:
            self.get_logger().error(f'Setpoint con modo desconocido: {msg.mode}')
            return

        command = Command(
            mode=mode,
            x=msg.position.x, y=msg.position.y, z=msg.position.z,
            vx=msg.velocity.x, vy=msg.velocity.y, vz=msg.velocity.z,
            yaw=msg.yaw,
        )
        if not self.drone.set_command(self.sim_time, command):
            # throttle: como mucho un aviso cada 2 s para no inundar el log.
            self.get_logger().warn(
                f'Setpoint ignorado: el dron está en modo {self.drone.mode}',
                throttle_duration_sec=2.0,
            )

    def _on_takeoff(self, request, response):
        response.success, response.message = self.drone.takeoff(
            request.height, request.duration
        )
        self.get_logger().info(f'/hk/takeoff -> {response.message}')
        return response

    def _on_land(self, request, response):
        response.success, response.message = self.drone.land(request.duration)
        self.get_logger().info(f'/hk/land -> {response.message}')
        return response

    def _on_emergency(self, request, response):
        del request
        self.drone.emergency_stop()
        response.success = True
        response.message = 'Emergencia: motores apagados, el dron cae'
        self.get_logger().warn('/hk/emergency -> motores apagados')
        return response

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------
    def _step(self):
        """Avanza la física un paso y registra los cambios de modo."""
        self.sim_time += self.dt
        self.drone.step(self.sim_time, self.dt)

        if self.drone.mode != self._last_mode:
            self.get_logger().info(f'Modo: {self._last_mode} -> {self.drone.mode}')
            self._last_mode = self.drone.mode

    def _publish_state(self):
        """Publica la pose real (sin ruido) y la 'estimada' (con ruido)."""
        stamp = self.get_clock().now().to_msg()
        self.truth_pub.publish(self._make_odom(stamp, noise_std=0.0))
        self.odom_pub.publish(self._make_odom(stamp, noise_std=self.pose_noise_std))

    def _make_odom(self, stamp, noise_std):
        """Convierte el DroneState en un nav_msgs/Odometry."""
        s = self.drone.state
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = s.x + random.gauss(0.0, noise_std)
        msg.pose.pose.position.y = s.y + random.gauss(0.0, noise_std)
        msg.pose.pose.position.z = s.z + random.gauss(0.0, noise_std)
        # Yaw -> cuaternión (rotación solo alrededor de z).
        msg.pose.pose.orientation.z = math.sin(s.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(s.yaw / 2.0)

        msg.twist.twist.linear.x = s.vx
        msg.twist.twist.linear.y = s.vy
        msg.twist.twist.linear.z = s.vz
        return msg


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
