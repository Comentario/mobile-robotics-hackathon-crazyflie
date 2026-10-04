"""Procesado de la ruta: sin zig-zag celda a celda (TP-30). Prioridad P1.

En modo direct puede reenviar la ruta sin cambios.

Suscribe:
  /hk/path                  nav_msgs/Path
  /hk/map                   nav_msgs/OccupancyGrid   para comprobar line of sight
Publica:
  /hk/path/processed        nav_msgs/Path
Parámetros:
  max_waypoint_spacing

Lógica: simplify.py.
"""
import rclpy
from rclpy.node import Node


class PathProcessor(Node):

    def __init__(self):
        super().__init__('path_processor')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('path_processor: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = PathProcessor()
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
