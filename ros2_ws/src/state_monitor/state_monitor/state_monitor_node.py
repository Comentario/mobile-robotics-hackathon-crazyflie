"""Single drone state and localization health (TP-13 to TP-17).

It does not implement an EKF: it consumes the onboard estimate of the Crazyflie (or
of the simulator).

Subscribes:
  /hk/adapter/odom          nav_msgs/Odometry
Publishes:
  /hk/state                 nav_msgs/Odometry               normalized state (frame map -> base_link)
  /hk/localization_health   hackathon_interfaces/LocalizationHealth  GOOD / DEGRADED / LOST
  /tf                       map -> base_link
Parameters:
  map_frame, base_frame, state_timeout, degraded_pose_age, max_position_jump,
  max_apparent_speed

Logic: health.py.

Implementation status
---------------------
  Done: /hk/state (TP-14), TF map -> base_link (TP-13), GOOD/DEGRADED/LOST health (TP-15).
  The z_min / z_max height band (TP-17) is enforced by safety_supervisor.
"""
import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from hackathon_interfaces.msg import LocalizationHealth

from .health import HealthConfig, LocalizationHealthMonitor

# Rate (Hz) at which /hk/localization_health is published. It is independent of the
# pose arrivals: that way, if they stop arriving, the state keeps being published (LOST).
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

        # Health thresholds (values in simulation.yaml).
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
        # TP-13: frame "broadcaster". Publishes on /tf where base_link is inside map.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/hk/adapter/odom', self._on_odom, 10)
        self.create_timer(1.0 / HEALTH_RATE, self._publish_health)

        self.get_logger().info(
            'state_monitor: /hk/state, TF map -> base_link and /hk/localization_health '
            f'(state_timeout={config.state_timeout} s, '
            f'degraded_pose_age={config.degraded_pose_age} s, '
            f'max_position_jump={config.max_position_jump} m)'
        )

    # ------------------------------------------------------------------
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id != self.map_frame or msg.child_frame_id != self.base_frame:
            return
        self.monitor.update(stamp, (p.x, p.y, p.z))
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        self.state_pub.publish(msg)
        self._publish_tf(msg)


    def _publish_tf(self, msg):
        """Publish on /tf the drone pose (base_link) inside the room (map) (TP-13)."""
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp          # same time as the pose measurement
        tf.header.frame_id = self.map_frame         # parent frame: the room
        tf.child_frame_id = self.base_frame         # child frame: the drone
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def _publish_health(self):
        """Evaluate the health now and publish it; log a warning when it changes."""
        now_msg = self.get_clock().now()
        now = now_msg.nanoseconds * 1e-9
        status, pose_age, reason = self.monitor.evaluate(now)

        msg = LocalizationHealth()
        msg.header.stamp = now_msg.to_msg()
        msg.header.frame_id = self.map_frame
        msg.status = status
        msg.pose_age = float(min(pose_age, 1e6))   # inf does not fit well in a float32 log field
        msg.reason = reason
        self.health_pub.publish(msg)

        if status != self._last_status:
            self.get_logger().warn(
                f'Localization health: {STATUS_NAMES.get(self._last_status, "-")} -> '
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
