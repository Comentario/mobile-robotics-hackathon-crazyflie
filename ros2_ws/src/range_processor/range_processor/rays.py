"""Multi-ranger ray geometry (TP-21). REP-103 axes: x forward, y left, z up."""
import math

SENSOR_DIRECTIONS = {
    'front': (1.0, 0.0, 0.0),
    'back': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
}


def _rotate_yaw(vector, yaw):
    x, y, z = vector
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*x - s*y, s*x + c*y, z)


def ray_in_map(pose, sensor, distance, sensor_offset, range_max):
    """Origin and end point of one ray in the map frame.

    pose: (x, y, z, yaw). sensor_offset: (x, y, z) in base_link (config/frames.yaml).
    REP-117 readings: +inf puts the end at range_max with hit=False (free space only);
    -inf (closer than range_min) is a hit at the sensor origin; NaN returns None.
    Returns (origin, end, hit) or None for an invalid reading.
    """
    if sensor not in SENSOR_DIRECTIONS:
        raise ValueError(f'Unknown sensor "{sensor}"')
    if len(pose) != 4 or len(sensor_offset) != 3:
        raise ValueError('pose must be (x, y, z, yaw) and sensor_offset (x, y, z)')
    if not all(math.isfinite(v) for v in (*pose, *sensor_offset)):
        raise ValueError('pose and sensor_offset must be finite')
    if not math.isfinite(range_max) or range_max <= 0.0:
        raise ValueError('range_max must be positive and finite')
    distance = float(distance)
    if math.isnan(distance):
        return None
    if distance == math.inf:
        length, hit = float(range_max), False
    elif distance == -math.inf:
        length, hit = 0.0, True
    else:
        if distance < 0.0:
            return None
        length, hit = min(distance, float(range_max)), distance <= range_max

    x, y, z, yaw = (float(v) for v in pose)
    offset = _rotate_yaw(sensor_offset, yaw)
    direction = _rotate_yaw(SENSOR_DIRECTIONS[sensor], yaw)
    origin = (x + offset[0], y + offset[1], z + offset[2])
    end = tuple(o + d*length for o, d in zip(origin, direction))
    return origin, end, hit
