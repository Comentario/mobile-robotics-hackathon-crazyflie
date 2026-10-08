"""Procesamiento de los cinco rayos del Multi-ranger + ToF inferior (TP-19 a TP-21, TP-36).

Suscribe:
  /hk/adapter/ranges        hackathon_interfaces/MultiRange  rangos crudos
  /hk/state                 nav_msgs/Odometry
Publica:
  /hk/ranges                hackathon_interfaces/MultiRange  rangos validados y filtrados
  /hk/ranges/closing_rate   hackathon_interfaces/MultiRange  d_dot por sensor (m/s, negativo = se acerca)
  /hk/ranges/markers        visualization_msgs/MarkerArray   rayos en el mapa para RViz
Parámetros:
  range_min, range_max, filter_window, max_valid_jump

Lógica: range_filter.py (validación, filtro, d_dot), rays.py (geometría de los rayos).

Estado de implementación
------------------------
  Hecho (TP-20): /hk/ranges y /hk/ranges/closing_rate. Un mensaje de salida por
                 cada mensaje de entrada, con el mismo header (stamp y frame base_link).
                 En closing_rate, NaN = d_dot no disponible (pocas muestras, sensor
                 sin retorno o salto recién aceptado).
  Pendiente (TP-21): /hk/ranges/markers con rays.py. Por eso /hk/state se declara
                 pero aún no se usa.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from hackathon_interfaces.msg import MultiRange

from .range_filter import SENSORS, RangeFilter, RangeFilterConfig


class RangeProcessor(Node):

    def __init__(self):
        super().__init__('range_processor')

        p = self.declare_parameter
        config = RangeFilterConfig(
            range_min=p('range_min', 0.02).value,
            range_max=p('range_max', 4.0).value,
            filter_window=p('filter_window', 3).value,
            max_valid_jump=p('max_valid_jump', 1.0).value,
            spike_confirm_time=p('spike_confirm_time', 0.3).value,
            invalid_hold=p('invalid_hold', 0.3).value,
        )
        self.config = config
        self.filter = RangeFilter(config)
        self._rejected = 0
        self.pose = None

        self.ranges_pub = self.create_publisher(MultiRange, '/hk/ranges', 10)
        self.rate_pub = self.create_publisher(MultiRange, '/hk/ranges/closing_rate', 10)

        self.create_subscription(MultiRange, '/hk/adapter/ranges', self._on_ranges, 10)
        self.create_subscription(Odometry, '/hk/state', self._on_state, 10)

        self.get_logger().info(
            f'range_processor listo: rango [{config.range_min}, {config.range_max}] m, '
            f'mediana de {config.filter_window}, spike > {config.max_valid_jump} m '
            f'confirmado tras {config.spike_confirm_time} s'
        )

    def _on_state(self, msg):
        self.pose = msg.pose.pose      # para los markers (TP-21, pendiente)

    def _on_ranges(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = {name: getattr(msg, name) for name in SENSORS}
        filtered = self.filter.update(t, raw)

        out = MultiRange()
        out.header = msg.header
        out.range_min = float(self.config.range_min)
        out.range_max = float(self.config.range_max)
        rate = MultiRange()
        rate.header = msg.header
        rate.range_min = out.range_min
        rate.range_max = out.range_max
        for name in SENSORS:
            setattr(out, name, float(filtered[name]))
            d_dot = self.filter.closing_rate(name)
            setattr(rate, name, float(d_dot) if d_dot is not None else math.nan)
        self.ranges_pub.publish(out)
        self.rate_pub.publish(rate)

        if self.filter.rejected_spikes != self._rejected:
            self._rejected = self.filter.rejected_spikes
            self.get_logger().warn(f'Spike de rango descartado (total: {self._rejected})')


def main(args=None):
    rclpy.init(args=args)
    node = RangeProcessor()
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
