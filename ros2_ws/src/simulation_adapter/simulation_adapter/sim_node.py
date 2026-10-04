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
import rclpy
from rclpy.node import Node


class SimulationAdapter(Node):

    def __init__(self):
        super().__init__('simulation_adapter')
        # TODO: declarar parámetros, publishers, subscribers y timers según el docstring.
        self.get_logger().warn('simulation_adapter: esqueleto sin implementar')


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
