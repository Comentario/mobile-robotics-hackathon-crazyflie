"""Simple kinematic drone model (TP-10).

It does not model quadrotor physics: the real onboard controller is approximated
with a first-order response towards the setpoint, limited in speed and acceleration.

This module is pure logic (rule 6 of docs/interfaces.md): it does not import rclpy
or ROS messages, so it can be tested with pytest without launching ROS. The node
(sim_node.py) translates the ROS messages into the Command class defined here.

Internal drone modes (attribute ``KinematicDrone.mode``)
--------------------------------------------------------
  'idle'       On the ground, motors stopped. Ignores setpoints.
  'takeoff'    Climbing to the requested height (linear ramp over ``duration``).
  'land'       Descending to the ground (linear ramp over ``duration``).
  'command'    Following the last received setpoint (POSITION / VELOCITY / HOVER).
  'hover'      Still at a fixed point. Entered when the takeoff ends (like the
               firmware high-level commander, which keeps generating setpoints).
               Without firmware_watchdog: also when the watchdog fires.
  'level_out'  Only with firmware_watchdog: no setpoints for command_watchdog
               (0.5 s). Roll/pitch to 0: keeps the height but keeps sliding with
               the horizontal velocity it had.
  'emergency'  Motors off: falls to z = 0 and no longer accepts any command.

Real firmware watchdog (firmware_watchdog=True)
-----------------------------------------------
  Copied from crazyflie-firmware 2025.02, src/modules/src/supervisor.c:
    COMMANDER_WDT_TIMEOUT_STABILIZE = 500 ms  -> WarningLevelOut state
    COMMANDER_WDT_TIMEOUT_SHUTDOWN  = 2000 ms -> ExceptFreeFall state (motors off)
  It only affects low-level setpoint streaming ('command' mode). The high-level
  commander (takeoff/land/hover after takeoff) generates its own setpoints and
  does not trigger the watchdog.

What every simulation step does (step)
--------------------------------------
  1. If in 'command' and the last setpoint is too old -> 'hover'.
  2. Compute a desired velocity according to the mode.
  3. Limit it to max_speed (3D norm).
  4. Limit the velocity change to max_accel * dt (maximum acceleration).
  5. Integrate the position: p = p + v * dt, never below the ground (z >= 0).
  6. Turn the yaw towards the target at a limited angular rate.
"""
import math
from dataclasses import dataclass

# Time constant (s) of the first-order response: with a position error e, the
# desired velocity is e / TIME_CONSTANT. The smaller, the more "aggressive" the
# tracking. 0.3 s gives smooth, believable motion.
TIME_CONSTANT = 0.3

# Maximum yaw rate (rad/s).
MAX_YAW_RATE = 1.5

# Fall acceleration in an emergency (m/s^2).
GRAVITY = 9.81

# Distance (m) below which the takeoff height, or the ground when landing, counts
# as reached.
ARRIVAL_TOLERANCE = 0.02


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


@dataclass
class Command:
    """Setpoint already translated from the ROS message (created by sim_node.py).

    mode:
      'position'  go to (x, y, z) with orientation yaw.
      'velocity'  move in the plane at velocity (vx, vy) keeping height z
                  (Setpoint.msg semantics: "follow velocity while keeping
                  position.z"). vz is ignored.
      'hover'     stay still where the drone is when the first HOVER arrives.
    """
    mode: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0


def _clamp_norm(vector, max_norm):
    """Scale a vector (tuple) so that its norm does not exceed max_norm."""
    norm = math.sqrt(sum(c * c for c in vector))
    if norm <= max_norm or norm == 0.0:
        return vector
    scale = max_norm / norm
    return tuple(c * scale for c in vector)


def _wrap_angle(angle):
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class KinematicDrone:

    def __init__(self, max_speed, max_accel, command_watchdog,
                 firmware_watchdog=False, shutdown_timeout=2.0):
        """command_watchdog: s without low-level setpoints before reacting.

        firmware_watchdog=False: "gentle" reaction (hover), as in the old simulator.
        firmware_watchdog=True:  like the real firmware: level_out after
        command_watchdog and fall (emergency) after shutdown_timeout.
        """
        self.state = DroneState()
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.command_watchdog = command_watchdog
        self.firmware_watchdog = firmware_watchdog
        self.shutdown_timeout = shutdown_timeout
        self.emergency_reason = ''

        self.mode = 'idle'
        self._t = 0.0                       # last time seen in step()
        self._command = None                # last accepted Command
        self._command_time = None           # time at which it arrived
        self._hover_point = (0.0, 0.0, 0.0)
        self._yaw_target = 0.0
        # Vertical takeoff/land ramp: (t0, x0, y0, z0, z1, duration)
        self._ramp = None

    # ------------------------------------------------------------------
    # Initial configuration
    # ------------------------------------------------------------------
    def reset_pose(self, x, y, z, yaw):
        """Place the drone at an initial pose (the scenario 'start')."""
        self.state = DroneState(x=x, y=y, z=z, yaw=yaw)
        self._yaw_target = yaw

    # ------------------------------------------------------------------
    # Inputs (called by the ROS node)
    # ------------------------------------------------------------------
    def set_command(self, t, setpoint):
        """Store the last Setpoint (POSITION, VELOCITY or HOVER mode) and its timestamp.

        Returns True if accepted and False if ignored (drone on the ground, in an
        emergency, or unknown mode).
        """
        if self.mode in ('idle', 'emergency'):
            return False
        if setpoint.mode not in ('position', 'velocity', 'hover'):
            return False

        # If HOVER was already being received, keep the hover point: that way a
        # continuous HOVER stream does not make the drone "drift".
        already_hovering = (
            self.mode == 'command'
            and self._command is not None
            and self._command.mode == 'hover'
        )

        if self.mode in ('takeoff', 'land'):
            # A low-level setpoint cuts the high-level manoeuvre (the firmware gives
            # priority to low-level streaming).
            self._ramp = None
        self._command = setpoint
        self._command_time = t
        self.mode = 'command'

        if setpoint.mode == 'hover':
            if not already_hovering:
                s = self.state
                self._hover_point = (s.x, s.y, s.z)
        else:
            self._yaw_target = setpoint.yaw
        return True

    def takeoff(self, height, duration):
        """Climb from the ground to z = height in 'duration' seconds.

        Returns (success, message), like the /hk/takeoff service.
        """
        if self.mode == 'emergency':
            return False, 'Emergency active: the drone does not accept commands'
        if self.mode != 'idle':
            return False, f'The drone is already flying (mode {self.mode})'
        if height <= 0.0 or duration <= 0.0:
            return False, 'height and duration must be positive'

        self.state.flying = True
        self._start_ramp(height, duration)
        self.mode = 'takeoff'
        return True, f'Taking off to {height:.2f} m in {duration:.1f} s'

    def land(self, duration):
        """Descend to the ground in 'duration' seconds and stop the motors on arrival."""
        if self.mode in ('idle', 'emergency'):
            return False, f'Cannot land in mode {self.mode}'
        if duration <= 0.0:
            return False, 'duration must be positive'

        self._start_ramp(0.0, duration)
        self._command = None
        self.mode = 'land'
        return True, f'Landing in {duration:.1f} s'

    def high_level_hover(self):
        """The high-level commander takes over: hover where the drone is.

        Used by the node when notify_setpoints_stop arrives (the firmware stops
        giving priority to low-level streaming).
        """
        if self.mode in ('command', 'level_out'):
            s = self.state
            self._enter_hover((s.x, s.y, s.z))
            self._command = None
            return True
        return False

    def emergency_stop(self, reason='emergency'):
        """Motors off: the drone falls (z -> 0) and stops accepting commands."""
        self.emergency_reason = reason
        self.mode = 'emergency'
        self.state.flying = False
        self._command = None
        self._ramp = None
        self.state.vx = 0.0
        self.state.vy = 0.0

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------
    def step(self, t, dt):
        """Advance the simulation by dt seconds and return the DroneState."""
        self._t = t
        s = self.state

        if self.mode == 'emergency':
            self._fall(dt)
            return s

        if self.mode == 'idle':
            s.vx = s.vy = s.vz = 0.0
            return s

        # 1. Watchdog: no recent low-level setpoints.
        if self.mode in ('command', 'level_out'):
            age = t - self._command_time
            if not self.firmware_watchdog:
                if age > self.command_watchdog:
                    self._enter_hover((s.x, s.y, s.z))
            elif age > self.shutdown_timeout:
                self.emergency_stop('watchdog')        # ExceptFreeFall
                self._fall(dt)
                return s
            elif age > self.command_watchdog and self.mode == 'command':
                self.mode = 'level_out'                # WarningLevelOut
                self._hover_point = (s.x, s.y, s.z)

        # 2-4. Desired velocity, limited in speed and acceleration.
        desired = _clamp_norm(self._desired_velocity(t), self.max_speed)
        delta = (desired[0] - s.vx, desired[1] - s.vy, desired[2] - s.vz)
        delta = _clamp_norm(delta, self.max_accel * dt)
        s.vx += delta[0]
        s.vy += delta[1]
        s.vz += delta[2]

        # 5. Integrate the position without going through the ground.
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z += s.vz * dt
        if s.z < 0.0:
            s.z = 0.0
            s.vz = max(s.vz, 0.0)

        # 6. Yaw with limited angular rate.
        yaw_error = _wrap_angle(self._yaw_target - s.yaw)
        max_step = MAX_YAW_RATE * dt
        s.yaw = _wrap_angle(s.yaw + max(-max_step, min(max_step, yaw_error)))

        self._check_maneuver_end(t)
        return s

    # ------------------------------------------------------------------
    # Internal functions
    # ------------------------------------------------------------------
    def _desired_velocity(self, t):
        """Velocity the 'controller' would want now, according to the mode."""
        s = self.state

        if self.mode in ('takeoff', 'land'):
            _, x0, y0, _, _, _ = self._ramp
            z_ref, vz_feedforward = self._ramp_reference(t)
            return self._towards((x0, y0, z_ref), vz_feedforward)

        if self.mode == 'hover':
            return self._towards(self._hover_point)

        if self.mode == 'level_out':
            # Roll/pitch = 0: no horizontal acceleration (keeps sliding).
            return (s.vx, s.vy, (self._hover_point[2] - s.z) / TIME_CONSTANT)

        # mode == 'command'
        cmd = self._command
        if cmd.mode == 'velocity':
            # XY: the requested velocity. Z: keep the setpoint height.
            return (cmd.vx, cmd.vy, (cmd.z - s.z) / TIME_CONSTANT)
        if cmd.mode == 'hover':
            return self._towards(self._hover_point)
        return self._towards((cmd.x, cmd.y, cmd.z))

    def _towards(self, target, vz_feedforward=0.0):
        """First-order response: velocity proportional to the error."""
        s = self.state
        return (
            (target[0] - s.x) / TIME_CONSTANT,
            (target[1] - s.y) / TIME_CONSTANT,
            (target[2] - s.z) / TIME_CONSTANT + vz_feedforward,
        )

    def _start_ramp(self, z_goal, duration):
        """Prepare a vertical ramp from the current height to z_goal."""
        s = self.state
        self._ramp = (self._t, s.x, s.y, s.z, z_goal, duration)

    def _ramp_reference(self, t):
        """Ramp reference height at time t and its velocity."""
        t0, _, _, z0, z1, duration = self._ramp
        progress = min(max((t - t0) / duration, 0.0), 1.0)
        z_ref = z0 + progress * (z1 - z0)
        vz_feedforward = (z1 - z0) / duration if progress < 1.0 else 0.0
        return z_ref, vz_feedforward

    def _check_maneuver_end(self, t):
        """Finish the takeoff (-> hover) or the landing (-> idle)."""
        if self._ramp is None or self.mode not in ('takeoff', 'land'):
            return
        t0, x0, y0, _, z1, duration = self._ramp
        ramp_done = t - t0 >= duration
        arrived = abs(self.state.z - z1) < ARRIVAL_TOLERANCE
        if not (ramp_done and arrived):
            return

        if self.mode == 'takeoff':
            self._enter_hover((x0, y0, z1))
        else:
            s = self.state
            s.z = 0.0
            s.vx = s.vy = s.vz = 0.0
            s.flying = False
            self._ramp = None
            self.mode = 'idle'

    def _enter_hover(self, point):
        self._hover_point = point
        self._ramp = None
        self.mode = 'hover'

    def _fall(self, dt):
        """Vertical free fall to the ground (emergency mode)."""
        s = self.state
        if s.z > 0.0:
            s.vz -= GRAVITY * dt
            s.z = max(0.0, s.z + s.vz * dt)
        if s.z == 0.0:
            s.vz = 0.0
