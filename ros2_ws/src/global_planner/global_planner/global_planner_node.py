"""Planificador global (TP-27 a TP-29).

planner_mode = direct: ruta recta pose -> goal, sin mapa (MVP / P0).
planner_mode = astar:  A* sobre /hk/map (P1).

Suscribe:
  /hk/map                   nav_msgs/OccupancyGrid       (solo en modo astar)
  /hk/state                 nav_msgs/Odometry
  /hk/goal                  geometry_msgs/PoseStamped
  /hk/replan_request        std_msgs/String              motivo, publicado por local_navigator
Publica:
  /hk/path                  nav_msgs/Path
  /hk/planner_status        hackathon_interfaces/PlannerStatus
Parámetros:
  planner_mode, planner_rate, replan_period, connectivity, unknown_policy, unknown_cost,
  proximity_weight, planning_timeout, max_path_deviation

Lógica: astar.py, replan.py.
"""
import rclpy
from rclpy.node import Node


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('global_planner: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
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
