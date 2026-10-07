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

Estado de implementación
------------------------
  Hecho: /hk/state (TP-14), TF map -> base_link (TP-13), salud GOOD/DEGRADED/LOST (TP-15).
  Pendiente: z_min / z_max (TP-17).
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from hackathon_interfaces.msg import LocalizationHealth

from .health import HealthConfig, LocalizationHealthMonitor

# Frecuencia (Hz) a la que se publica /hk/localization_health. Es independiente de
# la llegada de poses: así, si dejan de llegar, el estado sigue publicándose (LOST).
HEALTH_RATE = 20.0

STATUS_NAMES = {
    LocalizationHealth.GOOD: 'GOOD',
    LocalizationHealth.DEGRADED: 'DEGRADED',
    LocalizationHealth.LOST: 'LOST',
}


class StateMonitor(Node):

    def __init__(self):
        super().__init__('state_monitor')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        # Umbrales de salud (valores en simulation.yaml).
        config = HealthConfig(
            state_timeout=self.declare_parameter('state_timeout', 0.3).value,
            degraded_pose_age=self.declare_parameter('degraded_pose_age', 0.15).value,
            max_position_jump=self.declare_parameter('max_position_jump', 0.3).value,
            max_apparent_speed=self.declare_parameter('max_apparent_speed', 2.0).value,
        )
        self.monitor = LocalizationHealthMonitor(config)
        self._last_status = None

        self.state_pub = self.create_publisher(Odometry, '/hk/state', 10)
        self.health_pub = self.create_publisher(
            LocalizationHealth, '/hk/localization_health', 10
        )
        # TP-13: "altavoz" de frames. Publica en /tf dónde está base_link dentro de map.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/hk/adapter/odom', self._on_odom, 10)
        self.create_timer(1.0 / HEALTH_RATE, self._publish_health)

        self.get_logger().info(
            'state_monitor: /hk/state, TF map -> base_link y /hk/localization_health '
            f'(state_timeout={config.state_timeout} s, '
            f'degraded_pose_age={config.degraded_pose_age} s, '
            f'max_position_jump={config.max_position_jump} m)'
        )

    # ------------------------------------------------------------------
    def _on_odom(self, msg):
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        self.state_pub.publish(msg)
        self._publish_tf(msg)

        # TP-15: pasar la pose al monitor de salud, con la hora de la MEDIDA.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.monitor.update(stamp, (p.x, p.y, p.z))

    def _publish_tf(self, msg):
        """Publica en /tf la pose del dron (base_link) dentro de la sala (map) (TP-13)."""
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp          # misma hora que la medida de pose
        tf.header.frame_id = self.map_frame         # frame padre: la sala
        tf.child_frame_id = self.base_frame         # frame hijo: el dron
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def _publish_health(self):
        """Evalúa la salud ahora y la publica; avisa en el log si cambia."""
        now_msg = self.get_clock().now()
        now = now_msg.nanoseconds * 1e-9
        status, pose_age, reason = self.monitor.evaluate(now)

        msg = LocalizationHealth()
        msg.header.stamp = now_msg.to_msg()
        msg.header.frame_id = self.map_frame
        msg.status = status
        msg.pose_age = float(min(pose_age, 1e6))   # inf no cabe bien en float32 de log
        msg.reason = reason
        self.health_pub.publish(msg)

        if status != self._last_status:
            self.get_logger().warn(
                f'Salud de localización: {STATUS_NAMES.get(self._last_status, "-")} -> '
                f'{STATUS_NAMES[status]} (reason="{reason}", pose_age={pose_age:.2f} s)'
            )
            self._last_status = status


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
