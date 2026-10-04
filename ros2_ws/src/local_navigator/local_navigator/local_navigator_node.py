"""Seguimiento de ruta + evitación reactiva -> setpoint (TP-32 a TP-36).

Ruta + pose + rangos -> decisión local -> setpoint. No sustituye al planner:
reacciona más rápido que un replan ante obstáculos cercanos o dinámicos.

Suscribe:
  /hk/path/processed        nav_msgs/Path
  /hk/state                 nav_msgs/Odometry
  /hk/ranges                hackathon_interfaces/MultiRange
  /hk/ranges/closing_rate   hackathon_interfaces/MultiRange
  /hk/localization_health   hackathon_interfaces/LocalizationHealth
Publica:
  /hk/cmd/nav               hackathon_interfaces/Setpoint   propuesta; safety_supervisor decide
  /hk/replan_request        std_msgs/String                 cuando la dirección queda bloqueada
Parámetros:
  control_rate, lookahead_distance, nominal_speed, degraded_speed, max_speed, max_accel,
  goal_slowdown_radius, warning_distance, critical_distance, ttc_threshold, z_nominal, z_min, z_max

Lógica: follower.py, avoidance.py.
"""
import rclpy
from rclpy.node import Node


class LocalNavigator(Node):

    def __init__(self):
        super().__init__('local_navigator')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('local_navigator: esqueleto sin implementar')


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
