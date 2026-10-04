"""Telemetría y métricas (TP-40). La grabación se hace con scripts/record_bag.sh.

Suscribe:
  /hk/state, /hk/ranges, /hk/mission_status, /hk/planner_status, /hk/safety_state,
  /hk/localization_health, /hk/sim/ground_truth (solo simulación)
Publica:
  /hk/metrics               std_msgs/String   JSON con el resumen acumulado
Escribe:
  <output_dir>/<fecha>_<escenario>.json al terminar la misión
Parámetros:
  output_dir, scenario_name

Lógica: metrics.py.
"""
import rclpy
from rclpy.node import Node


class Telemetry(Node):

    def __init__(self):
        super().__init__('telemetry')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('telemetry: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = Telemetry()
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
