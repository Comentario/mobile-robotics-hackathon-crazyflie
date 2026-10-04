"""Mission Manager: qué waypoint toca y en qué fase está la misión (TP-26).

No decide cómo evitar obstáculos (eso es local_navigator).

Suscribe:
  /hk/state                 nav_msgs/Odometry
  /hk/localization_health   hackathon_interfaces/LocalizationHealth
  /hk/safety_state          hackathon_interfaces/SafetyState
  /hk/planner_status        hackathon_interfaces/PlannerStatus
Publica:
  /hk/goal                  geometry_msgs/PoseStamped        waypoint activo
  /hk/mission_status        hackathon_interfaces/MissionStatus
Clientes de servicio:
  /hk/takeoff, /hk/land
Servicios propios:
  /hk/mission/start, /hk/mission/pause, /hk/mission/resume, /hk/mission/abort   std_srvs/Trigger
Parámetros:
  mission_file, waypoint_tolerance, waypoint_timeout, z_nominal, takeoff_duration,
  land_duration, autostart

Lógica: mission.py.
"""
import rclpy
from rclpy.node import Node


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('mission_manager: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
