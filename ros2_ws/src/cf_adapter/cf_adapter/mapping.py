"""Conversiones entre mensajes de Crazyswarm2 y la API propia (TP-08, TP-09).

Mantener aquí todos los detalles de Crazyswarm2 (nombres, orden de rangos, unidades).
"""


def topic(robot_name, suffix):
    """Nombre completo de un topic de Crazyswarm2, p. ej. topic('cf231', 'pose')."""
    return f'/{robot_name}/{suffix}'


def scan_to_ranges(scan_ranges):
    """Convierte los rangos del LaserScan de Crazyswarm2 a dict sensor -> m.

    TODO(TP-08): confirmar el orden de los sensores (front/left/back/right) y cómo
    representa Crazyswarm2 una lectura sin retorno; convertir a REP-117.
    """
    raise NotImplementedError


def setpoint_to_position_cmd(setpoint):
    """Setpoint propio (modo POSITION o HOVER) -> campos de cmd_position (x, y, z, yaw en grados?).

    TODO(TP-08): confirmar unidades de yaw y frecuencia mínima de streaming.
    """
    raise NotImplementedError


def setpoint_to_hover_cmd(setpoint, current_yaw):
    """Setpoint propio (modo VELOCITY) -> cmd_hover (vx, vy en marco del cuerpo, yaw rate, z)."""
    raise NotImplementedError
