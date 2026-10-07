"""Safety Supervisor: único nodo que publica comandos hacia el adapter (TP-37 a TP-39).

Estados: NORMAL, SLOW, HOVER, LAND, ABORT. Prioridad sobre misión y navegación.

Suscribe:
  /hk/cmd/nav               hackathon_interfaces/Setpoint
  /hk/state                 nav_msgs/Odometry
  /hk/ranges                hackathon_interfaces/MultiRange
  /hk/ranges/closing_rate   hackathon_interfaces/MultiRange
  /hk/localization_health   hackathon_interfaces/LocalizationHealth
  /hk/planner_status        hackathon_interfaces/PlannerStatus
Publica:
  /hk/cmd/safe              hackathon_interfaces/Setpoint   comando filtrado (o hover)
  /hk/safety_state          hackathon_interfaces/SafetyState
Clientes de servicio:
  /hk/land, /hk/emergency
Parámetros:
  state_timeout, range_timeout, command_timeout, ranges_required, critical_distance,
  ttc_threshold, z_min, z_max, geofence, lost_policy, hover_timeout

Lógica: supervisor.py (máquina de estados), watchdog.py, geofence.py.

Cómo funciona el nodo
---------------------
  1. Cada mensaje que llega (estado, rangos, salud, comandos) actualiza los datos
     guardados y "da de comer" a su watchdog.
  2. Un temporizador a SAFETY_RATE Hz evalúa la máquina de estados, publica
     /hk/safety_state y, al ENTRAR en LAND o ABORT, llama una vez a /hk/land o
     /hk/emergency.
  3. Cada comando de /hk/cmd/nav se filtra según el estado actual y se publica en
     /hk/cmd/safe. Si el navegador deja de publicar, el supervisor tampoco publica:
     el watchdog del adapter/firmware mantiene el hover, y este nodo pasa a HOVER
     (y a LAND tras hover_timeout si el dron está en el aire).

Pendiente
---------
  TTC (TP-36): /hk/ranges/closing_rate se escucha, pero min_ttc es siempre inf
  hasta acordar el convenio de signo con range_processor.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from hackathon_interfaces.msg import (
    LocalizationHealth, MultiRange, PlannerStatus, SafetyState, Setpoint,
)
from hackathon_interfaces.srv import Land

from .geofence import Geofence
from .supervisor import (
    AIRBORNE_Z, LAND, ABORT, STATE_NAMES, SafetyInputs, SafetySupervisor, SupervisorConfig,
    min_finite,
)
from .watchdog import StreamWatchdog

SAFETY_RATE = 20.0       # Hz a los que se evalúa y publica el estado de seguridad
GEOFENCE_MARGIN = 0.1    # m de tolerancia antes de considerar al dron FUERA de la geofence
LAND_DURATION = 2.0      # s del aterrizaje pedido por seguridad


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor')

        # --- Parámetros (valores en simulation.yaml) ---------------------------
        p = self.declare_parameter
        state_timeout = p('state_timeout', 0.3).value
        range_timeout = p('range_timeout', 0.3).value
        command_timeout = p('command_timeout', 0.5).value
        self.z_min = p('z_min', 0.2).value
        self.z_max = p('z_max', 1.5).value
        fence = list(p('geofence', [-2.0, -2.0, 0.0, 2.0, 2.0, 1.5]).value)
        config = SupervisorConfig(
            hover_timeout=p('hover_timeout', 3.0).value,
            lost_policy=p('lost_policy', 'hover_then_land').value,
            critical_distance=p('critical_distance', 0.3).value,
            ttc_threshold=p('ttc_threshold', 1.5).value,
            ranges_required=p('ranges_required', True).value,
        )

        # --- Lógica pura --------------------------------------------------------
        self.supervisor = SafetySupervisor(config)
        self.geofence = Geofence(*fence)
        self.state_wd = StreamWatchdog(state_timeout)
        self.range_wd = StreamWatchdog(range_timeout)
        self.cmd_wd = StreamWatchdog(command_timeout)

        # --- Últimos datos recibidos -------------------------------------------
        self.position = None                    # (x, y, z) del dron
        self.localization = LocalizationHealth.LOST
        self.min_range = math.inf
        self.planner_no_path = False
        self._no_path_value = getattr(PlannerStatus, 'NO_PATH', None)
        self._last_state = None
        self._speed_scale = 1.0

        # --- Publishers ---------------------------------------------------------
        self.cmd_pub = self.create_publisher(Setpoint, '/hk/cmd/safe', 10)
        self.state_pub = self.create_publisher(SafetyState, '/hk/safety_state', 10)

        # --- Subscribers --------------------------------------------------------
        self.create_subscription(Setpoint, '/hk/cmd/nav', self._on_nav, 10)
        self.create_subscription(Odometry, '/hk/state', self._on_state, 10)
        self.create_subscription(MultiRange, '/hk/ranges', self._on_ranges, 10)
        self.create_subscription(
            LocalizationHealth, '/hk/localization_health', self._on_health, 10)
        self.create_subscription(
            PlannerStatus, '/hk/planner_status', self._on_planner, 10)

        # --- Clientes de servicio -----------------------------------------------
        self.land_client = self.create_client(Land, '/hk/land')
        self.emergency_client = self.create_client(Trigger, '/hk/emergency')

        self.create_timer(1.0 / SAFETY_RATE, self._evaluate)

        self.get_logger().info(
            f'safety_supervisor listo: command_timeout={command_timeout} s, '
            f'hover_timeout={config.hover_timeout} s, lost_policy={config.lost_policy}, '
            f'ranges_required={config.ranges_required}, geofence={fence}, '
            f'z=[{self.z_min}, {self.z_max}]'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Entradas
    # ------------------------------------------------------------------
    def _on_state(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)
        self.state_wd.stamp(self._now())

    def _on_ranges(self, msg):
        self.min_range = min_finite([msg.front, msg.back, msg.left, msg.right])
        self.range_wd.stamp(self._now())

    def _on_health(self, msg):
        self.localization = msg.status

    def _on_planner(self, msg):
        self.planner_no_path = (
            self._no_path_value is not None and msg.status == self._no_path_value
        )

    def _on_nav(self, msg):
        """Filtra el comando del navegador según el estado actual y lo publica."""
        self.cmd_wd.stamp(self._now())

        setpoint = {
            'mode': msg.mode,
            'position': (msg.position.x, msg.position.y, msg.position.z),
            'velocity': (msg.velocity.x, msg.velocity.y, msg.velocity.z),
            'yaw': msg.yaw,
        }
        out = self.supervisor.filter_command(
            setpoint, self.supervisor.state, self._speed_scale, self.position,
            geofence=self.geofence, z_limits=(self.z_min, self.z_max),
        )
        if out is None:
            return

        safe = Setpoint()
        safe.header = msg.header
        safe.mode = out['mode']
        safe.position.x, safe.position.y, safe.position.z = out['position']
        safe.velocity.x, safe.velocity.y, safe.velocity.z = out['velocity']
        safe.yaw = out['yaw']
        self.cmd_pub.publish(safe)

    # ------------------------------------------------------------------
    # Evaluación periódica
    # ------------------------------------------------------------------
    def _evaluate(self):
        now = self._now()
        z = self.position[2] if self.position else 0.0

        # Dron en el suelo y sin órdenes: el navegador anterior ya no cuenta.
        # El silencio vuelve a significar "la misión aún no empezó", y así el dron
        # puede despegar de nuevo tras un aterrizaje de seguridad.
        if z <= AIRBORNE_Z and self.cmd_wd.received_any() and self.cmd_wd.stale(now):
            self.cmd_wd.last_stamp = None
        inputs = SafetyInputs(
            now=now,
            localization=self.localization,
            state_stale=self.state_wd.stale(now),
            ranges_stale=self.range_wd.stale(now),
            # Los comandos solo "vencen" si alguna vez llegaron: antes de la primera
            # orden del navegador el dron está esperando, no ha perdido nada.
            command_stale=self.cmd_wd.received_any() and self.cmd_wd.stale(now),
            min_horizontal_range=self.min_range if not self.range_wd.stale(now) else math.inf,
            min_ttc=math.inf,
            z=z,
            planner_no_path=self.planner_no_path,
            outside_geofence=(self.position is not None
                              and not self.geofence.contains(self.position,
                                                             margin=GEOFENCE_MARGIN)),
        )
        state, speed_scale, reason = self.supervisor.evaluate(inputs)
        self._speed_scale = speed_scale

        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.speed_scale = float(speed_scale)
        msg.reason = reason
        self.state_pub.publish(msg)

        if state != self._last_state:
            previous = STATE_NAMES[self._last_state] if self._last_state is not None else '-'
            self.get_logger().warn(
                f'Seguridad: {previous} -> {STATE_NAMES[state]} (reason="{reason}")'
            )
            if state == LAND:
                self._call_land()
            elif state == ABORT:
                self._call_emergency()
            self._last_state = state

    def _call_land(self):
        if not self.land_client.service_is_ready():
            self.get_logger().error('LAND pedido pero /hk/land no está disponible')
            return
        request = Land.Request()
        request.duration = LAND_DURATION
        self.land_client.call_async(request)
        self.get_logger().warn('Seguridad: aterrizaje solicitado por /hk/land')

    def _call_emergency(self):
        if not self.emergency_client.service_is_ready():
            self.get_logger().error('ABORT pedido pero /hk/emergency no está disponible')
            return
        self.emergency_client.call_async(Trigger.Request())
        self.get_logger().error('Seguridad: EMERGENCIA solicitada por /hk/emergency')


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
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
