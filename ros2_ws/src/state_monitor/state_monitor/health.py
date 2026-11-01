"""Localization health monitor (TP-15) and speed policy (TP-16).

Pure logic (rule 6 of interfaces.md): no rclpy, tested with pytest.

What it answers: "can I trust the position I am receiving?"

How it decides the state (evaluate), in this order of priority
--------------------------------------------------------------
  1. No pose ever arrived                         -> LOST      reason 'stale'
  2. The last pose had NaN or infinity            -> LOST      reason 'nan'
  3. pose_age > state_timeout                     -> LOST      reason 'stale'
  4. pose_age > degraded_pose_age                 -> DEGRADED  reason 'stale'
  5. There was a jump in the last JUMP_HOLD s     -> DEGRADED  reason 'jump'
  6. Impossible speed within JUMP_HOLD s          -> DEGRADED  reason 'overspeed'
  7. All good                                     -> GOOD      reason ''

pose_age (age of the pose)
--------------------------
  Seconds since the last measurement with an increasing timestamp. An identical
  position with a new timestamp is valid: the drone may be standing still.
  Dropout, delay and freeze are detected when the old timestamp is kept.
  A source that repeats positions but fabricates new timestamps needs an
  independent check: it cannot be told apart from a stationary drone.
"""
import math
from dataclasses import dataclass

GOOD, DEGRADED, LOST = 0, 1, 2   # same as hackathon_interfaces/LocalizationHealth

# Time (s) DEGRADED is held after a jump or an impossible speed, so the rest of the
# stack gets to see it and it is not a single-sample warning.
JUMP_HOLD = 1.0

# Window (s) to measure the apparent speed. Between two consecutive samples (0.02 s)
# pose noise and PC or radio delays give false speeds (1 cm of noise / 0.005 s =
# 2 m/s). Over 0.1 s the noise barely matters.
SPEED_WINDOW = 0.1


@dataclass
class HealthConfig:
    state_timeout: float        # s without pose -> LOST
    degraded_pose_age: float    # s without pose -> DEGRADED
    max_position_jump: float    # m between consecutive samples
    max_apparent_speed: float   # m/s


class LocalizationHealthMonitor:

    def __init__(self, config):
        self.config = config
        self._last_position = None     # last received position
        self._last_stamp = None        # time of that position
        self._last_fresh_stamp = None  # time of the last NEW pose (see docstring)
        self._nan = False              # the last pose was invalid
        self._jump_time = None         # time of the last detected jump
        self._overspeed_time = None    # time of the last impossible speed
        self._speed_ref = None         # (t, position) reference for the speed

    def update(self, t, position):
        """Record a new pose (t in s, position = (x, y, z)). Detects NaN/inf and jumps.

        t is the MEASUREMENT time (header.stamp), not the arrival time.
        """
        # NaN / infinity: the pose is unusable. It is not stored as a reference.
        if not all(math.isfinite(c) for c in position):
            self._nan = True
            return
        if not math.isfinite(t) or (self._last_stamp is not None and t <= self._last_stamp):
            return
        self._nan = False

        if self._last_position is not None:
            distance = math.dist(position, self._last_position)

            # Jump: between two consecutive samples (a real jump is instantaneous).
            if distance > self.config.max_position_jump:
                self._jump_time = t
                self._speed_ref = (t, tuple(position))   # the jump does not count as speed

            # Apparent speed: over a window of SPEED_WINDOW s.
            ref_t, ref_pos = self._speed_ref
            dt = t - ref_t
            if dt >= SPEED_WINDOW:
                if math.dist(position, ref_pos) / dt > self.config.max_apparent_speed:
                    self._overspeed_time = t
                self._speed_ref = (t, tuple(position))
            elif dt < 0.0:
                self._speed_ref = (t, tuple(position))   # older pose: restart

            # New pose = increasing timestamp, even if the position does not change.
            if self._last_stamp is None or t > self._last_stamp:
                self._last_fresh_stamp = t
        else:
            self._last_fresh_stamp = t
            self._speed_ref = (t, tuple(position))

        self._last_position = tuple(position)
        self._last_stamp = t

    def evaluate(self, now):
        """Return (status, pose_age, reason). reason: '', 'stale', 'jump', 'nan', 'overspeed'."""
        if self._last_fresh_stamp is None:
            return LOST, math.inf, 'stale'

        pose_age = max(0.0, now - self._last_fresh_stamp)

        if self._nan:
            return LOST, pose_age, 'nan'
        if pose_age > self.config.state_timeout:
            return LOST, pose_age, 'stale'
        if pose_age > self.config.degraded_pose_age:
            return DEGRADED, pose_age, 'stale'
        if self._jump_time is not None and now - self._jump_time < JUMP_HOLD:
            return DEGRADED, pose_age, 'jump'
        if self._overspeed_time is not None and now - self._overspeed_time < JUMP_HOLD:
            return DEGRADED, pose_age, 'overspeed'
        return GOOD, pose_age, ''


def speed_limit_for(status, nominal_speed, degraded_speed):
    """GOOD -> nominal, DEGRADED -> reduced, LOST -> 0 (do not navigate)."""
    if status == GOOD:
        return nominal_speed
    if status == DEGRADED:
        return degraded_speed
    return 0.0
