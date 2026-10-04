"""Geometría de los rayos del Multi-ranger (TP-21). Ejes REP-103: x adelante, y izquierda, z arriba."""

SENSOR_DIRECTIONS = {
    'front': (1.0, 0.0, 0.0),
    'back': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
}


def ray_in_map(pose, sensor, distance, sensor_offset, range_max):
    """Origen y extremo de un rayo en el frame map.

    pose: (x, y, z, yaw). sensor_offset: (x, y, z) en base_link (config/frames.yaml).
    Si distance es +inf, el extremo se sitúa a range_max y hit=False (solo espacio libre).
    Devuelve (origin, end, hit).
    """
    raise NotImplementedError
