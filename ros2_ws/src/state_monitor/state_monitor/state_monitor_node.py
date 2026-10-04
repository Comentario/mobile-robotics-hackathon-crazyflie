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
from nav_msgs.msg import Odometry
from rclpy.node import Node


class StateMonitor(Node):

    def __init__(self):
        super().__init__('state_monitor')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        # Por ahora (smoke test de TP-06) solo reenvía /hk/adapter/odom a /hk/state con los frames
        # normalizados. La validación y la salud de la localización llegan con TP-14 y TP-15.
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.state_pub = self.create_publisher(Odometry, '/hk/state', 10)
        self.create_subscription(Odometry, '/hk/adapter/odom', self._on_odom, 10)
        self.get_logger().warn('state_monitor: solo reenvía la pose (TP-06 smoke test)')

    def _on_odom(self, msg):
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        self.state_pub.publish(msg)


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
