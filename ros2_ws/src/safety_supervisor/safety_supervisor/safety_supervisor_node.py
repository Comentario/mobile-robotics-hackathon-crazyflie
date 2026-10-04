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
"""
import rclpy
from rclpy.node import Node


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('safety_supervisor: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
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
