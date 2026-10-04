"""Evaluador de escenarios de la test matrix T01–T16 (TP-18, TP-25, TP-31, TP-41).

Se lanza con `simulation.launch.py evaluate:=true`.

Suscribe:
  /hk/metrics               std_msgs/String   (de telemetry)
  /hk/mission_status        hackathon_interfaces/MissionStatus
Al terminar (misión COMPLETED/ABORTED o timeout del escenario):
  compara las métricas con `expected` del escenario, imprime PASS/FAIL y lo guarda en
  <output_dir>/results.csv. Sale con código 0 (pass) o 1 (fail).
Parámetros:
  scenario_file, output_dir

Lógica: scenario.py.
"""
import rclpy
from rclpy.node import Node


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('hackathon_tests')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('hackathon_tests: esqueleto sin implementar')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
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
