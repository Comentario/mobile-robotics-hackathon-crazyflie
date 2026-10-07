"""Multi-ranger sintético: offsets respecto a base_link, yaw y REP-117."""
import math

# Misma geometría nominal que config/frames.yaml (TP-13 medirá el hardware).
SENSORS = {
    'front': ((0.02, 0, 0.03), (1, 0, 0)),
    'back': ((-0.02, 0, 0.03), (-1, 0, 0)),
    'left': ((0, 0.02, 0.03), (0, 1, 0)),
    'right': ((0, -0.02, 0.03), (0, -1, 0)),
    'up': ((0, 0, 0.04), (0, 0, 1)),
    'down': ((0, 0, -0.01), (0, 0, -1)),
}


def sample_ranges(world, state, t, range_min, range_max, noise_std=0.0, rng=None):
    if not (0 <= range_min < range_max) or not all(math.isfinite(v) for v in (range_min, range_max, noise_std)) or noise_std < 0:
        raise ValueError('Límites/ruido de rangos inválidos')
    if noise_std and rng is None:
        raise ValueError('El ruido requiere un generador aleatorio')
    c, s = math.cos(state.yaw), math.sin(state.yaw)
    rotate = lambda v: (c*v[0]-s*v[1], s*v[0]+c*v[1], v[2])
    result = {}
    for name, (offset, direction) in SENSORS.items():
        offset = rotate(offset)
        origin = tuple(p+d for p, d in zip((state.x, state.y, state.z), offset))
        value = world.raycast(origin, rotate(direction), range_max, t)
        if value < range_min:
            value = -math.inf
        elif math.isfinite(value):
            value += rng.gauss(0, noise_std) if noise_std else 0
            value = -math.inf if value < range_min else math.inf if value > range_max else value
        result[name] = value
    return result
