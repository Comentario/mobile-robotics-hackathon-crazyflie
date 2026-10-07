"""Pure conversion helpers used by the Crazyflie adapter."""

import math


def topic(robot_name, suffix):
    """Build a normalized Crazyswarm2 topic or service name."""
    robot = robot_name.strip('/')
    endpoint = suffix.strip('/')
    return f'/{robot}/{endpoint}'


def yaw_to_quaternion(yaw_rad):
    """Convert a yaw angle in radians to a quaternion."""
    half_yaw = 0.5 * yaw_rad
    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def twist_transform(quaternion, linear_frame='body', angular_units='radians'):
    """6x6 transform to standard ROS body-frame twist in SI units."""
    if linear_frame not in ('body', 'world') or angular_units not in ('radians', 'degrees'):
        raise ValueError('Unsupported odometry twist convention')
    x, y, z, w = quaternion
    rotation = (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )
    matrix = [[0.0]*6 for _ in range(6)]
    for i in range(3):
        for j in range(3):
            matrix[i][j] = rotation[j][i] if linear_frame == 'world' else float(i == j)
        matrix[i+3][i+3] = math.pi/180 if angular_units == 'degrees' else 1.0
    return matrix


def transform_twist(values, covariance, matrix):
    """Apply the same convention change to twist and its 6x6 covariance."""
    if not all(map(math.isfinite, (*values, *covariance))):
        raise ValueError('Non-finite odometry twist or covariance')
    velocity = tuple(sum(matrix[i][j]*values[j] for j in range(6)) for i in range(6))
    cov = [sum(matrix[i][k]*covariance[6*k+l]*matrix[j][l]
               for k in range(6) for l in range(6)) for i in range(6) for j in range(6)]
    return velocity, cov


def seconds_to_duration_parts(duration_s):
    """Convert floating-point seconds to ROS sec/nanosec fields."""
    if not math.isfinite(duration_s) or duration_s < 0.0 or duration_s >= 2**31:
        raise ValueError('Duration must be finite, non-negative and fit int32 seconds')

    sec = int(duration_s)
    nanosec = int(round((duration_s - sec) * 1_000_000_000))

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    if sec >= 2**31:
        raise ValueError('Duration exceeds int32 seconds')

    return sec, nanosec


def scan_to_ranges(scan_ranges, angle_min=-math.pi, angle_increment=math.pi / 2):
    """Map four cardinal rays using LaserScan angles; preserve REP-117 values.

    Installed cflib server publishes [back, right, front, left], starting at -pi.
    Reject rotated, missing or duplicate rays rather than guessing orientation.
    """
    if len(scan_ranges) != 4 or not all(map(math.isfinite, (angle_min, angle_increment))):
        raise ValueError('Expected four cardinal LaserScan rays')
    rays = {}
    names = ('front', 'left', 'back', 'right')
    for index, distance in enumerate(scan_ranges):
        angle = (angle_min + index * angle_increment) % (2 * math.pi)
        cardinal = round(angle / (math.pi / 2)) % 4
        error = (angle - cardinal * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
        if abs(error) > 1e-4 or names[cardinal] in rays:
            raise ValueError('LaserScan rays must be distinct cardinal directions')
        rays[names[cardinal]] = float(distance)
    return rays


class ReferenceTranslator:
    """Translate position/XY velocity/hover to an absolute FullState reference.

    XY velocity is integrated into a position reference, not native velocity-only
    control. Hover latches measured position on entry and keeps it until exit.
    """

    def __init__(self, initial_dt=0.05, max_dt=0.1):
        if not 0 < initial_dt <= max_dt or not math.isfinite(max_dt):
            raise ValueError('Invalid reference integration periods')
        self.initial_dt = initial_dt
        self.max_dt = max_dt
        self.reset()

    def reset(self):
        self.mode = None
        self.reference = None
        self.last_time = None
        self.hover_yaw = None

    def translate(self, mode, position, velocity, yaw, measured_position, measured_yaw, now):
        if mode not in (0, 1, 2):
            raise ValueError('Unsupported setpoint mode')
        values = (*position, *velocity, yaw, *measured_position, measured_yaw, now)
        if not all(map(math.isfinite, values)):
            raise ValueError('Non-finite setpoint or state')
        if self.last_time is not None and now < self.last_time:
            self.reset()
            raise ValueError('Clock moved backwards')
        if mode == 0:
            reference, feedforward, target_yaw = tuple(position), (0.0, 0.0, 0.0), yaw
        elif mode == 1:
            if abs(velocity[2]) > 1e-6:
                raise ValueError('VELOCITY maintains position.z; vertical velocity must be zero')
            dt = self.initial_dt if self.last_time is None else now - self.last_time
            if dt > self.max_dt:
                self.reset()
                raise ValueError('Velocity reference stream expired')
            origin = self.reference if self.mode == 1 else measured_position
            reference = (origin[0] + velocity[0] * dt, origin[1] + velocity[1] * dt, position[2])
            feedforward, target_yaw = (velocity[0], velocity[1], 0.0), yaw
        else:
            reference = self.reference if self.mode == 2 else tuple(measured_position)
            target_yaw = self.hover_yaw if self.mode == 2 else measured_yaw
            feedforward = (0.0, 0.0, 0.0)
            self.hover_yaw = target_yaw
        self.mode, self.reference, self.last_time = mode, reference, now
        return reference, feedforward, target_yaw
