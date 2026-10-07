"""Simple kinematic model used by the development simulator.

The model does not reproduce quadrotor physics. It approximates the onboard
controller as a velocity-limited and acceleration-limited response toward the
requested setpoint.
"""

from dataclasses import dataclass
import math


MODE_POSITION = 0
MODE_VELOCITY = 1
MODE_HOVER = 2


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


class KinematicDrone:
    """Minimal kinematic Crazyflie approximation."""

    def __init__(self, max_speed, max_accel, command_watchdog):
        self.state = DroneState()

        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.command_watchdog = float(command_watchdog)

        self._mode = MODE_HOVER

        self._target_position = [0.0, 0.0, 0.0]
        self._target_velocity = [0.0, 0.0, 0.0]
        self._target_yaw = 0.0

        self._last_command_time = None
        self._watchdog_triggered = False

        self._high_level_command = None
        self._emergency = False

        self._speed_limit = self.max_speed

    @staticmethod
    def _limit_vector(vector, limit):
        """Limit the Euclidean norm of a 3D vector."""
        norm = math.sqrt(sum(value * value for value in vector))

        if norm == 0.0 or norm <= limit:
            return list(vector)

        scale = limit / norm
        return [value * scale for value in vector]

    def _enter_hover(self):
        """Hold the current position."""
        self._mode = MODE_HOVER

        self._target_position = [
            self.state.x,
            self.state.y,
            self.state.z,
        ]

        self._target_velocity = [0.0, 0.0, 0.0]
        self._target_yaw = self.state.yaw
        self._speed_limit = self.max_speed

    def set_command(self, t, setpoint):
        """Store a POSITION, VELOCITY, or HOVER streaming command.

        Returns True when the command is accepted and False when commands are
        disabled because the vehicle is not flying or emergency mode is active.
        """
        if self._emergency or not self.state.flying:
            return False

        mode = int(setpoint.mode)

        if mode not in (MODE_POSITION, MODE_VELOCITY, MODE_HOVER):
            raise ValueError(f'Unsupported setpoint mode: {mode}')

        self._high_level_command = None
        self._last_command_time = float(t)
        self._watchdog_triggered = False
        self._speed_limit = self.max_speed
        self._target_yaw = float(setpoint.yaw)

        if mode == MODE_POSITION:
            self._mode = MODE_POSITION

            self._target_position = [
                float(setpoint.position.x),
                float(setpoint.position.y),
                float(setpoint.position.z),
            ]

        elif mode == MODE_VELOCITY:
            self._mode = MODE_VELOCITY

            self._target_velocity = [
                float(setpoint.velocity.x),
                float(setpoint.velocity.y),
                0.0,
            ]

            # The project Setpoint contract uses position.z as altitude target.
            self._target_position[2] = float(setpoint.position.z)

        else:
            self._enter_hover()

        return True

    def takeoff(self, height, duration):
        """Start a simple vertical takeoff maneuver."""
        height = float(height)
        duration = float(duration)

        if self._emergency:
            return False

        if height <= 0.0:
            raise ValueError('Takeoff height must be positive')

        if duration <= 0.0:
            raise ValueError('Takeoff duration must be positive')

        self.state.flying = True

        self._high_level_command = 'takeoff'
        self._mode = MODE_POSITION

        self._target_position = [
            self.state.x,
            self.state.y,
            height,
        ]

        self._target_yaw = self.state.yaw

        requested_speed = abs(height - self.state.z) / duration

        self._speed_limit = min(
            self.max_speed,
            max(0.05, requested_speed),
        )

        self._last_command_time = None
        self._watchdog_triggered = False

        return True

    def land(self, duration):
        """Start a simple vertical landing maneuver."""
        duration = float(duration)

        if duration <= 0.0:
            raise ValueError('Landing duration must be positive')

        if self._emergency:
            return False

        if not self.state.flying:
            return True

        self._high_level_command = 'land'
        self._mode = MODE_POSITION

        self._target_position = [
            self.state.x,
            self.state.y,
            0.0,
        ]

        self._target_yaw = self.state.yaw

        requested_speed = self.state.z / duration

        self._speed_limit = min(
            self.max_speed,
            max(0.05, requested_speed),
        )

        self._last_command_time = None
        self._watchdog_triggered = False

        return True

    def emergency_stop(self):
        """Disable commands and let the simulated vehicle fall to the floor."""
        self._emergency = True
        self._high_level_command = None
        self._last_command_time = None

        self._target_velocity = [0.0, 0.0, 0.0]

    def _desired_velocity(self):
        """Compute the velocity requested by the active control mode."""
        if self._mode in (MODE_POSITION, MODE_HOVER):
            error = [
                self._target_position[0] - self.state.x,
                self._target_position[1] - self.state.y,
                self._target_position[2] - self.state.z,
            ]

            desired = [
                2.0 * error[0],
                2.0 * error[1],
                2.0 * error[2],
            ]

            return self._limit_vector(
                desired,
                self._speed_limit,
            )

        desired = [
            self._target_velocity[0],
            self._target_velocity[1],
            2.0 * (self._target_position[2] - self.state.z),
        ]

        return self._limit_vector(
            desired,
            self.max_speed,
        )

    def step(self, t, dt):
        """Advance the simulation by dt seconds."""
        t = float(t)
        dt = float(dt)

        if dt <= 0.0:
            return self.state

        if self._emergency:
            self.state.vx = 0.0
            self.state.vy = 0.0

            self.state.vz = max(
                self.state.vz - self.max_accel * dt,
                -self.max_speed,
            )

            self.state.z = max(
                0.0,
                self.state.z + self.state.vz * dt,
            )

            if self.state.z <= 1e-6:
                self.state.z = 0.0
                self.state.vz = 0.0
                self.state.flying = False

            return self.state

        if not self.state.flying:
            self.state.z = 0.0
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.vz = 0.0

            return self.state

        # High-level takeoff/land commands do not require streaming updates.
        if (
            self._high_level_command is None
            and self._last_command_time is not None
            and not self._watchdog_triggered
            and t - self._last_command_time > self.command_watchdog
        ):
            self._enter_hover()
            self._watchdog_triggered = True

        desired_velocity = self._desired_velocity()

        current_velocity = [
            self.state.vx,
            self.state.vy,
            self.state.vz,
        ]

        velocity_delta = [
            desired_velocity[index] - current_velocity[index]
            for index in range(3)
        ]

        velocity_delta = self._limit_vector(
            velocity_delta,
            self.max_accel * dt,
        )

        self.state.vx += velocity_delta[0]
        self.state.vy += velocity_delta[1]
        self.state.vz += velocity_delta[2]

        limited_velocity = self._limit_vector(
            [
                self.state.vx,
                self.state.vy,
                self.state.vz,
            ],
            self.max_speed,
        )

        self.state.vx = limited_velocity[0]
        self.state.vy = limited_velocity[1]
        self.state.vz = limited_velocity[2]

        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt

        self.state.z = max(
            0.0,
            self.state.z + self.state.vz * dt,
        )

        # Yaw dynamics are intentionally simplified for the development model.
        self.state.yaw = self._target_yaw

        position_error = math.sqrt(
            (self._target_position[0] - self.state.x) ** 2
            + (self._target_position[1] - self.state.y) ** 2
            + (self._target_position[2] - self.state.z) ** 2
        )

        if (
            self._high_level_command == 'takeoff'
            and position_error < 0.01
        ):
            self._high_level_command = None
            self._enter_hover()

        if (
            self._high_level_command == 'land'
            and self.state.z < 0.01
        ):
            self.state.z = 0.0
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.vz = 0.0
            self.state.flying = False

            self._high_level_command = None
            self._enter_hover()

        return self.state