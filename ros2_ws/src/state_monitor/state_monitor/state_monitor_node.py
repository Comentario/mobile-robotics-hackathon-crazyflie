"""Estado único del dron y salud de la localización (TP-13 a TP-17).

No implementa un EKF: consume la estimación onboard del Crazyflie (o del simulador).

Suscribe:
  /hk/adapter/odom          nav_msgs/Odometry
Publica:
  /hk/state                 nav_msgs/Odometry               estado normalizado (frame map -> base_link)
  /hk/localization_health   hackathon_interfaces/LocalizationHealth  GOOD / DEGRADED / LOST
  /tf                       map -> base_link
Parámetros:
  map_frame, base_frame, state_timeout, degraded_pose_age, max_position_jump,
  max_apparent_speed, z_min, z_max

Lógica: health.py.
"""
import rclpy
from rclpy.node import Node


class StateMonitor(Node):

    def __init__(self):
        super().__init__('state_monitor')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('state_monitor: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
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
