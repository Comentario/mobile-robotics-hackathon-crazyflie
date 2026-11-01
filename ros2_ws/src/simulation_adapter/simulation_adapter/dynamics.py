"""Simple kinematic drone model (TP-10).

It does not model quadrotor physics: the real onboard controller is approximated
with a first-order response towards the setpoint, limited in speed and acceleration.
"""
from dataclasses import dataclass, replace
import math


def position_step(position, target, max_speed, dt):
    """First TP-10 step: straight motion at maximum speed, without acceleration.

    position/target: XYZ tuples in metres; max_speed: m/s; dt: seconds.
    Returns (new position, XYZ velocity). Never overshoots the target.
    Not connected to KinematicDrone or to the ROS node.
    """
    if len(position) != 3 or len(target) != 3:
        raise ValueError('position and target must have three coordinates')
    if not all(math.isfinite(value) for value in (*position, *target, max_speed, dt)):
        raise ValueError('Coordinates, speed and time must be finite')
    if max_speed <= 0 or dt <= 0:
        raise ValueError('max_speed and dt must be positive')

    displacement = tuple(goal - current for current, goal in zip(position, target))
    distance = math.hypot(*displacement)
    if not math.isfinite(distance):
        raise ValueError('The distance between coordinates is too large')
    if distance == 0:
        return tuple(position), (0.0, 0.0, 0.0)

    # The length of the whole motion is limited, not each axis separately.
    step_distance = min(distance, max_speed * dt)
    fraction = step_distance / distance
    movement = tuple(value * fraction for value in displacement)
    next_position = tuple(target) if step_distance == distance else tuple(
        current + change for current, change in zip(position, movement))
    velocity = tuple(change / dt for change in movement)
    return next_position, velocity


@dataclass
class DroneState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    flying: bool = False
    yaw_rate: float = 0.0


def accelerated_position_step(state, target, max_speed, max_accel, dt, response_time=1.0):
    """TP-10, step 2: position tracking with limited speed and acceleration.

    Returns a new DroneState without modifying the given one. response_time (s)
    reduces the desired velocity when approaching the target. The stopping distance
    also limits that velocity. It does not change yaw or flying nor simulate motors.
    A reference that changes abruptly can cause overshoot due to inertia; the
    position is never forced to the target nor the velocity zeroed instantly.
    """
    if not isinstance(state, DroneState) or len(target) != 3:
        raise ValueError('A DroneState and an XYZ target are required')
    position = (state.x, state.y, state.z)
    velocity = (state.vx, state.vy, state.vz)
    values = (*position, *velocity, state.yaw, *target, max_speed, max_accel, dt, response_time)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('State, target and parameters must be finite')
    if min(max_speed, max_accel, dt, response_time) <= 0 or dt > response_time:
        raise ValueError('Parameters must be positive; dt must not exceed response_time')
    if math.hypot(*velocity) > max_speed + 1e-12:
        raise ValueError('The initial velocity exceeds max_speed')

    displacement = tuple(goal - current for current, goal in zip(position, target))
    distance = math.hypot(*displacement)
    if not math.isfinite(distance):
        raise ValueError('The distance between coordinates is too large')
    if distance == 0:
        desired_velocity = (0.0, 0.0, 0.0)
    else:
        # v^2 = 2*a*d: speed from which the drone can still stop within distance d.
        desired_speed = min(max_speed, distance / response_time, math.sqrt(2 * max_accel * distance))
        desired_velocity = tuple(value / distance * desired_speed for value in displacement)

    velocity_change = tuple(desired - current for desired, current in zip(desired_velocity, velocity))
    change_norm = math.hypot(*velocity_change)
    fraction = min(1.0, max_accel * dt / change_norm) if change_norm else 0.0
    next_velocity = tuple(current + change * fraction for current, change in zip(velocity, velocity_change))
    # Average of initial and final velocity: constant acceleration during dt.
    next_position = tuple(current + (before + after) * 0.5 * dt
                          for current, before, after in zip(position, velocity, next_velocity))
    return replace(state, x=next_position[0], y=next_position[1], z=next_position[2],
                   vx=next_velocity[0], vy=next_velocity[1], vz=next_velocity[2])


@dataclass(frozen=True)
class MotionCommand:
    """ROS-independent reference: 0=POSITION, 1=VELOCITY XY, 2=HOVER."""
    mode: int
    position: tuple = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    velocity: tuple = (0.0, 0.0, 0.0)


class KinematicDrone:

    def __init__(self, max_speed, max_accel, command_watchdog, initial_state=None,
                 response_time=1.0, position_tolerance=0.01, settle_speed=0.02,
                 gravity=9.81, max_yaw_rate=1.5):
        """command_watchdog: s without setpoints after which the drone hovers.

        This is a policy of our own simulator, not a reproduction of the real
        watchdog: TP-08 documents that the firmware does not guarantee hover on timeout.
        """
        parameters = (max_speed, max_accel, command_watchdog, response_time,
                      position_tolerance, settle_speed, gravity, max_yaw_rate)
        if not all(math.isfinite(value) and value > 0 for value in parameters):
            raise ValueError('Limits and times must be finite and positive')
        self.state = replace(initial_state) if initial_state is not None else DroneState()
        if self.state.z < 0:
            raise ValueError('The initial state cannot be below the ground')
        accelerated_position_step(self.state, (self.state.x, self.state.y, self.state.z),
                                  max_speed, max_accel, min(0.01, response_time), response_time)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.command_watchdog = command_watchdog
        self.response_time = response_time
        self.position_tolerance = position_tolerance
        self.settle_speed = settle_speed
        self.gravity = gravity
        self.max_yaw_rate = max_yaw_rate
        self.target_yaw = self.state.yaw
        self.velocity_reference = (0.0, 0.0, 0.0)
        self.phase = 'FLYING' if self.state.z > 0 else 'GROUND'
        self.state.flying = self.phase == 'FLYING'
        self.target = (self.state.x, self.state.y, self.state.z)
        self.mode = 2
        self.last_command_time = None
        self.maneuver_start = self.target
        self.maneuver_goal = self.target
        self.maneuver_elapsed = 0.0
        self.maneuver_duration = 0.0

    def set_command(self, t, setpoint):
        """Accept POSITION, VELOCITY XY and HOVER only in free flight."""
        if self.phase != 'FLYING':
            raise ValueError(f'Comando bloqueado durante {self.phase}')
        if not math.isfinite(t) or len(setpoint.position) != 3:
            raise ValueError('Invalid timestamp and position')
        if not all(math.isfinite(value) for value in (*setpoint.position, setpoint.yaw)):
            raise ValueError('Non-finite reference')
        if self.last_command_time is not None and t < self.last_command_time:
            raise ValueError('Out-of-order command')
        if setpoint.mode in (0, 1):
            if setpoint.position[2] < 0:
                raise ValueError('The target cannot be below the ground')
            if setpoint.mode == 1:
                if len(setpoint.velocity) != 3 or not all(math.isfinite(v) for v in setpoint.velocity):
                    raise ValueError('Invalid velocity')
                if abs(setpoint.velocity[2]) > 1e-9:
                    raise ValueError('VELOCITY keeps position.z; velocity.z must be zero')
                norm = math.hypot(*setpoint.velocity)
                scale = min(1.0, self.max_speed / norm) if norm else 1.0
                self.velocity_reference = tuple(v * scale for v in setpoint.velocity)
                self.target = (self.state.x, self.state.y, setpoint.position[2])
            else:
                self.target = tuple(setpoint.position)
            self.target_yaw = setpoint.yaw
        elif setpoint.mode == 2:
            if self.mode != 2:
                self.target = (self.state.x, self.state.y, self.state.z)
                self.target_yaw = self.state.yaw
        else:
            raise ValueError('Modo desconocido')
        self.mode = setpoint.mode
        self.last_command_time = t

    def _begin_maneuver(self, phase, height, duration):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('The duration must be finite and positive')
        self.phase = phase
        self.maneuver_start = (self.state.x, self.state.y, self.state.z)
        self.maneuver_goal = (self.state.x, self.state.y, height)
        self.maneuver_duration = duration
        self.maneuver_elapsed = 0.0
        self.last_command_time = None
        self.mode = 2
        self.target_yaw = self.state.yaw

    def takeoff(self, height, duration):
        if self.phase != 'GROUND':
            raise ValueError('Takeoff is only accepted from the ground')
        if not math.isfinite(height) or height <= 0:
            raise ValueError('The height must be finite and positive')
        self._begin_maneuver('TAKING_OFF', height, duration)

    def land(self, duration):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('The duration must be finite and positive')
        if self.phase == 'EMERGENCY':
            raise ValueError('Emergency latched: restart the simulator')
        if self.phase in ('GROUND', 'LANDING'):
            return  # Repeated requests neither restart nor extend the landing.
        self._begin_maneuver('LANDING', 0.0, duration)

    def emergency_stop(self):
        """Simulated cut-off: gravity fall to the ground, locked until restart."""
        self.phase = 'EMERGENCY'
        self.last_command_time = None

    def step(self, t, dt):
        """Advance the simulation by dt seconds and return the DroneState."""
        if not math.isfinite(t) or not math.isfinite(dt) or dt <= 0 or dt > self.response_time:
            raise ValueError('Invalid time; dt must be positive and <= response_time')
        state = self.state
        if self.phase == 'GROUND':
            return replace(state)
        if self.phase == 'EMERGENCY':
            vz = state.vz - self.gravity * dt
            z = state.z + (state.vz + vz) * 0.5 * dt
            self.state = replace(state, x=state.x + state.vx * dt,
                                 y=state.y + state.vy * dt, z=max(0.0, z), vz=vz, yaw_rate=0.0)
            if z <= 0:
                self.state = replace(self.state, vx=0.0, vy=0.0, vz=0.0, flying=False)
            return replace(self.state)
        maneuver = self.phase in ('TAKING_OFF', 'LANDING')
        if maneuver:
            self.maneuver_elapsed += dt
            progress = min(1.0, self.maneuver_elapsed / self.maneuver_duration)
            # Smooth reference: zero reference velocity and acceleration at both ends.
            fraction = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
            self.target = tuple(start + (goal - start) * fraction
                                for start, goal in zip(self.maneuver_start, self.maneuver_goal))
        elif self.last_command_time is not None:
            age = t - self.last_command_time
            if age > self.command_watchdog or age < 0:
                self.target = (state.x, state.y, state.z)
                self.mode = 2
                self.target_yaw = state.yaw
                self.last_command_time = None
        if not maneuver and self.mode == 1:
            desired = (*self.velocity_reference[:2],
                       (self.target[2] - state.z) / self.response_time)
            norm = math.hypot(*desired)
            desired = tuple(v * min(1.0, self.max_speed / norm) for v in desired) if norm else desired
            velocity = (state.vx, state.vy, state.vz)
            change = tuple(v-u for u, v in zip(velocity, desired))
            norm = math.hypot(*change)
            scale = min(1.0, self.max_accel * dt / norm) if norm else 0.0
            next_v = tuple(u+v*scale for u, v in zip(velocity, change))
            pos = tuple(p+(u+v)*dt/2 for p, u, v in zip((state.x, state.y, state.z), velocity, next_v))
            self.state = replace(state, x=pos[0], y=pos[1], z=pos[2],
                                 vx=next_v[0], vy=next_v[1], vz=next_v[2])
        else:
            self.state = accelerated_position_step(state, self.target, self.max_speed,
                                                   self.max_accel, dt, self.response_time)
        error = (self.target_yaw - state.yaw + math.pi) % (2 * math.pi) - math.pi
        turn = max(-self.max_yaw_rate*dt, min(self.max_yaw_rate*dt, error))
        self.state.yaw = (state.yaw + turn + math.pi) % (2 * math.pi) - math.pi
        self.state.yaw_rate = turn / dt
        self.state.flying = self.state.z > 0
        position = (self.state.x, self.state.y, self.state.z)
        speed = math.hypot(self.state.vx, self.state.vy, self.state.vz)
        distance = math.dist(position, self.maneuver_goal) if maneuver else math.inf
        settled = distance <= self.position_tolerance and speed <= self.settle_speed
        if maneuver and self.maneuver_elapsed >= self.maneuver_duration and settled:
            if self.phase == 'TAKING_OFF':
                self.phase = 'FLYING'
                self.target = self.maneuver_goal
            else:
                # Ground contact within tolerance: zero velocity.
                self.phase = 'GROUND'
                self.state = replace(self.state, z=0.0, vx=0.0, vy=0.0, vz=0.0, flying=False)
                self.target = (self.state.x, self.state.y, 0.0)
        elif self.state.z <= 0:
            self.state = replace(self.state, z=0.0, vx=0.0, vy=0.0, vz=0.0, flying=False)
            if self.phase != 'TAKING_OFF':
                self.phase = 'GROUND'
                self.target = (self.state.x, self.state.y, 0.0)
                self.mode = 2
                self.last_command_time = None
        return replace(self.state)
