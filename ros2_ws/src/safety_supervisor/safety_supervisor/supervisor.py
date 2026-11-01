"""Safety state machine (TP-37).

Priority: ABORT > LAND > HOVER > SLOW > NORMAL. It is evaluated every cycle; a more
severe state is only left once its cause has gone away (with hysteresis).

Pure logic (rule 6 of interfaces.md): no rclpy, tested with pytest.

What evaluate() decides every cycle (from most to least severe)
---------------------------------------------------------------
  ABORT   No rule triggers it yet (reserved). Once active it is never left.
  LAND    A "loss" cause (localization LOST, stale state or stale commands) lasts
          longer than hover_timeout AND the drone is airborne.
          Only with lost_policy = 'hover_then_land'.
  HOVER   Localization LOST, stale state, stale commands, stale ranges
          (if ranges_required), obstacle too close (see "Stopping distance"),
          drone outside the geofence, or the planner has no path.
  SLOW    Localization DEGRADED, or TTC below ttc_threshold.
  NORMAL  None of the above.

Stopping distance
-----------------
  A drone flying towards a wall at speed v does not stop instantly: it first needs
  reaction_time to react (range filter, supervisor cycle, radio) and then brakes
  with deceleration max_decel. It travels
      d_stop = v * reaction_time + v^2 / (2 * max_decel)
  So the node does not compare the range with critical_distance directly, but the
  "clearance" = range - d_stop (effective_clearance), where v is the speed at which
  THAT sensor sees the obstacle approach (-d_dot). Moving parallel to a wall does
  not count as approaching it.
  T03 test in simulation: at 1 m/s with critical_distance 0.3 m and without this
  term, the drone touched the wall.

Hysteresis
----------
  Moving to a MORE severe state is immediate. Returning to a LESS severe one requires
  the cause to be gone for RECOVERY_TIME consecutive seconds. That way a fault that
  comes and goes does not make the drone "blink" between HOVER and NORMAL.

What filter_command() does with the navigator command
-----------------------------------------------------
  NORMAL  Passes it through (clamped to the geofence and to z_min/z_max).
  SLOW    Passes it through, but slower:
            VELOCITY: velocity x speed_scale.
            POSITION: target moved towards the drone, at most slow_max_step m
                      (and at most speed_scale x distance). With a distant target,
                      only scaling the distance did not slow anything down.
  HOVER   Replaces it with "stay still here".
  LAND / ABORT  Drops it (None): the landing or emergency has already been requested
                through a service.
"""
import math
from dataclasses import dataclass

NORMAL, SLOW, HOVER, LAND, ABORT = range(5)   # same as hackathon_interfaces/SafetyState
STATE_NAMES = ('NORMAL', 'SLOW', 'HOVER', 'LAND', 'ABORT')

# Setpoint modes (same as hackathon_interfaces/Setpoint).
MODE_POSITION, MODE_VELOCITY, MODE_HOVER = 0, 1, 2

# GOOD / DEGRADED / LOST (same as hackathon_interfaces/LocalizationHealth).
GOOD, DEGRADED, LOST = 0, 1, 2

RECOVERY_TIME = 1.0   # s without cause before returning to a less severe state
AIRBORNE_Z = 0.1      # m: above this height the drone is considered airborne


@dataclass
class SafetyInputs:
    now: float
    localization: int           # GOOD / DEGRADED / LOST
    state_stale: bool
    ranges_stale: bool
    command_stale: bool
    min_horizontal_range: float
    min_ttc: float
    z: float
    planner_no_path: bool
    outside_geofence: bool


@dataclass
class SupervisorConfig:
    hover_timeout: float = 3.0          # s in HOVER due to a loss before landing
    lost_policy: str = 'hover_then_land'  # 'hover_then_land' | 'hover'
    critical_distance: float = 0.3      # m: closer obstacle -> HOVER
    ttc_threshold: float = 1.5          # s: lower TTC -> SLOW
    ranges_required: bool = True        # no ranges -> HOVER
    slow_speed_scale: float = 0.5       # speed factor in SLOW
    slow_max_step: float = 0.15         # m: in SLOW, POSITION target at most this far
    reaction_time: float = 0.3          # s: total delay until braking starts
    max_decel: float = 2.0              # m/s^2: deceleration (= simulator max_accel)


class SafetySupervisor:
    def __init__(self, config):
        self.config = config
        self.state = NORMAL
        self.reason = 'startup'
        self._loss_since = None      # since when a "loss" cause has lasted
        self._calm_since = None      # since when the cause is less severe than the state

    # ------------------------------------------------------------------
    def evaluate(self, inputs):
        """Return (state, speed_scale, reason)."""
        wanted, reason = self._wanted_state(inputs)

        if self.state == ABORT:
            pass                                    # ABORT is never left
        elif wanted >= self.state:
            # Equal or more severe: applied immediately.
            self.state, self.reason = wanted, reason
            self._calm_since = None
        else:
            # Less severe: only after RECOVERY_TIME consecutive seconds without the cause.
            if self._calm_since is None:
                self._calm_since = inputs.now
            if inputs.now - self._calm_since >= RECOVERY_TIME:
                self.state, self.reason = wanted, reason
                self._calm_since = None

        return self.state, self._speed_scale(self.state), self.reason

    def _wanted_state(self, i):
        """State the inputs would request right now, without hysteresis."""
        c = self.config

        # "Loss" causes: we do not know where we are or nobody is commanding us.
        loss = []
        if i.localization == LOST:
            loss.append('localization_lost')
        if i.state_stale:
            loss.append('state_timeout')
        if i.command_stale:
            loss.append('command_timeout')

        if loss:
            if self._loss_since is None:
                self._loss_since = i.now
        else:
            self._loss_since = None

        airborne = i.z > AIRBORNE_Z
        if (loss and airborne and c.lost_policy == 'hover_then_land'
                and i.now - self._loss_since > c.hover_timeout):
            return LAND, loss[0]

        hover = list(loss)
        if c.ranges_required and i.ranges_stale:
            hover.append('ranges_timeout')
        if i.min_horizontal_range < c.critical_distance:
            hover.append('obstacle_close')
        if i.outside_geofence:
            hover.append('outside_geofence')
        if i.planner_no_path:
            hover.append('no_path')
        if hover:
            return HOVER, hover[0]

        if i.localization == DEGRADED:
            return SLOW, 'localization_degraded'
        if i.min_ttc < c.ttc_threshold:
            return SLOW, 'ttc'
        return NORMAL, ''

    def _speed_scale(self, state):
        if state == NORMAL:
            return 1.0
        if state == SLOW:
            return self.config.slow_speed_scale
        return 0.0

    # ------------------------------------------------------------------
    def filter_command(self, setpoint, state, speed_scale, current_position,
                       geofence=None, z_limits=None):
        """Apply the state to the setpoint: scale velocity, turn into hover or drop it.

        setpoint: dict {'mode', 'position': (x,y,z), 'velocity': (vx,vy,vz), 'yaw'}.
        current_position: current (x, y, z) of the drone, or None if still unknown.
        geofence: Geofence object (or None). z_limits: (z_min, z_max) (or None).
        Returns a dict with the same format, or None if nothing must be published.
        """
        if state in (LAND, ABORT):
            return None

        if state == HOVER or setpoint['mode'] == MODE_HOVER:
            if current_position is None:
                return None
            return {
                'mode': MODE_HOVER,
                'position': tuple(current_position),
                'velocity': (0.0, 0.0, 0.0),
                'yaw': setpoint['yaw'],
            }

        out = {
            'mode': setpoint['mode'],
            'position': tuple(setpoint['position']),
            'velocity': tuple(setpoint['velocity']),
            'yaw': setpoint['yaw'],
        }

        # SLOW: slower.
        if speed_scale < 1.0:
            if out['mode'] == MODE_VELOCITY:
                out['velocity'] = tuple(v * speed_scale for v in out['velocity'])
            elif current_position is not None:
                delta = [t - c for c, t in zip(current_position, out['position'])]
                dist = math.sqrt(sum(d * d for d in delta))
                step = min(speed_scale * dist, self.config.slow_max_step)
                k = step / dist if dist > 0.0 else 0.0
                out['position'] = tuple(c + k * d for c, d in zip(current_position, delta))

        # Geofence (TP-39) and height band (TP-17): the target never leaves the box.
        if geofence is not None:
            out['position'] = geofence.clamp(out['position'])
        if z_limits is not None:
            x, y, z = out['position']
            out['position'] = (x, y, min(max(z, z_limits[0]), z_limits[1]))

        # In VELOCITY, do not push outwards if the drone is already at the border.
        if out['mode'] == MODE_VELOCITY and geofence is not None and current_position:
            v = list(out['velocity'])
            for k in range(3):
                if current_position[k] <= geofence.lower[k] and v[k] < 0:
                    v[k] = 0.0
                if current_position[k] >= geofence.upper[k] and v[k] > 0:
                    v[k] = 0.0
            out['velocity'] = tuple(v)

        return out


def stopping_distance(speed, config):
    """Metres the drone travels until it stops when flying at 'speed' m/s."""
    v = max(0.0, speed)
    return v * config.reaction_time + v * v / (2.0 * config.max_decel)


def effective_clearance(distance, closing_rate, config):
    """Range minus the stopping distance towards that obstacle (REP-117).

    closing_rate: sensor d_dot (m/s, negative = approaching) or None/NaN.
    Returns inf if the reading is unusable (+inf or NaN) and 0 if it is -inf (touching).
    """
    if distance == -math.inf:
        return 0.0
    if not math.isfinite(distance):
        return math.inf
    approach = 0.0
    if closing_rate is not None and math.isfinite(closing_rate) and closing_rate < 0.0:
        approach = -closing_rate
    return max(0.0, distance - stopping_distance(approach, config))


def min_finite(values):
    """Minimum distance of a list of REP-117 ranges, or inf if there is none.

    -inf (too close) counts as 0 m: it is the MOST dangerous case, it is not ignored.
    +inf (no return) and NaN (invalid) are ignored.
    """
    usable = [0.0 if v == -math.inf else v for v in values]
    finite = [v for v in usable if math.isfinite(v) and v >= 0.0]
    return min(finite) if finite else math.inf
