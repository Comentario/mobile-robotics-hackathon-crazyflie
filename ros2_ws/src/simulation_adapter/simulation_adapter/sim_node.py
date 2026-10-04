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
"""
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SimulationAdapter(Node):

    def __init__(self):
        super().__init__('simulation_adapter')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        # Por ahora (smoke test de TP-06) solo publica una pose fija en /hk/adapter/odom.
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        state_rate = self.declare_parameter('state_rate', 50.0).value
        scenario_file = self.declare_parameter('scenario_file', '').value

        self.start = [0.0, 0.0, 0.0, 0.0]   # x, y, z, yaw
        if scenario_file:
            with open(scenario_file, encoding='utf-8') as f:
                self.start = (yaml.safe_load(f) or {}).get('start', self.start)

        self.odom_pub = self.create_publisher(Odometry, '/hk/adapter/odom', 10)
        self.create_timer(1.0 / state_rate, self._publish_odom)
        self.get_logger().warn('simulation_adapter: solo publica una pose fija (TP-06 smoke test)')

    def _publish_odom(self):
        x, y, z, yaw = self.start
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(msg)


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
