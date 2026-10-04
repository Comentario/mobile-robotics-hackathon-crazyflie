"""Adaptador de hardware: traduce la API propia a Crazyswarm2 y viceversa (TP-08, TP-09, VS-5).

Ningún otro paquete debe usar topics o servicios de Crazyswarm2.

Suscribe (propio):
  /hk/cmd/safe              hackathon_interfaces/Setpoint
Publica (propio):
  /hk/adapter/odom          nav_msgs/Odometry
  /hk/adapter/ranges        hackathon_interfaces/MultiRange
Servicios (propios, mismos que simulation_adapter):
  /hk/takeoff, /hk/land, /hk/emergency
Lado Crazyswarm2 (a verificar en TP-08 y con `ros2 topic list` durante un vuelo):
  /<robot>/pose, /<robot>/odom, /<robot>/scan   requieren activar firmware_logging en crazyflies.yaml
  /<robot>/cmd_position, /<robot>/cmd_hover, ... setpoints de bajo nivel (streaming)
  /<robot>/takeoff, /<robot>/land, /all/emergency
Parámetros:
  robot_name, command_mode (position | hover), down_tof_available

Lógica: mapping.py (conversiones puras, testeables sin hardware).
"""
import rclpy
from rclpy.node import Node


class CrazyflieAdapter(Node):

    def __init__(self):
        super().__init__('cf_adapter')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('cf_adapter: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieAdapter()
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
