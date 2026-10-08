"""Conversión de los rangos de Crazyswarm2 al formato del equipo (TP-09).

Lógica pura (regla 6 de interfaces.md): sin rclpy, se prueba con pytest.

De dónde vienen los rangos en Crazyswarm2 1.0.7 (verificado en el código fuente,
crazyflie_server_cpp/src/crazyflie_server.cpp, on_logging_scan, y su gemelo
crazyflie_server_py/crazyflie_server.py, _log_scan_data_callback):

  /<robot>/scan  sensor_msgs/LaserScan, se activa con 'scan' en
                 firmware_logging.default_topics de crazyflies.yaml.
                 Variables del firmware: range.front, range.left, range.back, range.right (mm).
                 ranges = [back, right, front, left]
                 angle_min = -pi, angle_increment = pi/2
                 range_min = 0.01, range_max = 3.49; un valor > 3.49 m se publica como +inf.

  up y down NO van en el scan. Se piden como custom topic en crazyflies.yaml:
      custom_topics:
        ranges_extra:
          frequency: 20
          vars: ["range.up", "range.zrange"]
  y llegan en /<robot>/ranges_extra (crazyflie_interfaces/LogDataGeneric),
  values = [range.up, range.zrange] en mm, en el orden de 'vars'.

Este módulo NO depende del orden del scan: usa el ángulo de cada rayo
(angle_min + i * angle_increment), así que sigue funcionando aunque
Crazyswarm2 cambie el orden en otra versión.
"""
import math

# Ángulo de cada sensor horizontal en base_link (REP-103: x adelante, y izquierda).
SCAN_SENSOR_ANGLES = {
    'front': 0.0,
    'left': math.pi / 2.0,
    'back': math.pi,
    'right': -math.pi / 2.0,
}

# Tolerancia para asignar un rayo del scan a un sensor (rad).
ANGLE_TOLERANCE = math.pi / 8.0


def _angle_diff(a, b):
    """Diferencia angular en [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def to_rep117(value_m, range_min, range_max):
    """Lectura en metros -> convención REP-117 (+inf sin retorno, -inf muy cerca)."""
    if math.isnan(value_m):
        return math.nan
    if value_m > range_max:
        return math.inf
    if value_m < range_min:
        return -math.inf
    return value_m


def scan_to_ranges(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan (listas/números) -> dict {'front','left','back','right'} en metros.

    Un sensor que no aparece en el scan queda como NaN (inválido).
    """
    out = {name: math.nan for name in SCAN_SENSOR_ANGLES}
    for i, value in enumerate(ranges):
        angle = angle_min + i * angle_increment
        for name, sensor_angle in SCAN_SENSOR_ANGLES.items():
            if abs(_angle_diff(angle, sensor_angle)) <= ANGLE_TOLERANCE:
                out[name] = to_rep117(float(value), range_min, range_max)
    return out


def extra_to_ranges(values_mm, range_min, range_max):
    """LogDataGeneric.values [range.up, range.zrange] en mm -> {'up', 'down'} en metros."""
    if len(values_mm) < 2:
        return {'up': math.nan, 'down': math.nan}
    return {
        'up': to_rep117(float(values_mm[0]) / 1000.0, range_min, range_max),
        'down': to_rep117(float(values_mm[1]) / 1000.0, range_min, range_max),
    }
