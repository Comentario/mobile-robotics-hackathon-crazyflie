"""Synthetic Multi-ranger: offsets relative to base_link, yaw and REP-117."""
import math

# Same nominal geometry as config/frames.yaml (TP-13 will measure the hardware).
SENSORS = {
    'front': ((0.02, 0, 0.03), (1, 0, 0)),
    'back': ((-0.02, 0, 0.03), (-1, 0, 0)),
    'left': ((0, 0.02, 0.03), (0, 1, 0)),
    'right': ((0, -0.02, 0.03), (0, -1, 0)),
    'up': ((0, 0, 0.04), (0, 0, 1)),
    'down': ((0, 0, -0.01), (0, 0, -1)),
}


def sample_ranges(world, state, t, range_min, range_max, noise_std=0.0, rng=None, orientation=None):
    if not (0 <= range_min < range_max) or not all(math.isfinite(v) for v in (range_min, range_max, noise_std)) or noise_std < 0:
        raise ValueError('Invalid range limits/noise')
    if noise_std and rng is None:
        raise ValueError('Noise requires a random generator')
    c, s = math.cos(state.yaw), math.sin(state.yaw)
    rotate = lambda v: (c*v[0]-s*v[1], s*v[0]+c*v[1], v[2])
    if orientation is not None:
        if len(orientation) != 4 or not all(math.isfinite(v) for v in orientation) or not math.isclose(math.hypot(*orientation),1,abs_tol=1e-6):
            raise ValueError('A unit quaternion is required')
        x,y,z,w = orientation
        def rotate(v):
            # q*v*q^-1; includes the SIL roll/pitch, without reducing them to yaw.
            tx,ty,tz = 2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])
            return (v[0]+w*tx+y*tz-z*ty, v[1]+w*ty+z*tx-x*tz, v[2]+w*tz+x*ty-y*tx)
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
