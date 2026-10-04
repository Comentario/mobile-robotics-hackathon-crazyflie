"""Mapa local: rolling occupancy grid 2D (TP-22 a TP-25). Prioridad P1.

Suscribe:
  /hk/ranges                hackathon_interfaces/MultiRange
  /hk/state                 nav_msgs/Odometry
Publica:
  /hk/map                   nav_msgs/OccupancyGrid  -1 unknown, 0 libre, 100 ocupado (ya inflado)
Parámetros:
  map_resolution, map_width, map_height, obstacle_inflation_radius, obstacle_decay_time, map_rate

Lógica: grid.py.
"""
import rclpy
from rclpy.node import Node


class LocalWorldModel(Node):

    def __init__(self):
        super().__init__('local_world_model')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('local_world_model: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = LocalWorldModel()
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
