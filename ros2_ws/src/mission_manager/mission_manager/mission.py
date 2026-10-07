"""Pure mission loading and state-machine logic for TP-26."""

from dataclasses import dataclass
import math

import yaml


IDLE = 0
PREFLIGHT = 1
TAKEOFF = 2
NAVIGATE = 3
LANDING = 4
COMPLETED = 5
PAUSED = 6
ABORTED = 7

HEALTH_GOOD = 0

SAFETY_NORMAL = 0
SAFETY_SLOW = 1
SAFETY_HOVER = 2
SAFETY_LAND = 3
SAFETY_ABORT = 4


@dataclass(frozen=True)
class Waypoint:
    """One mission waypoint expressed in the map frame."""

    x: float
    y: float
    z: float


def load_mission(path):
    """Load and validate a waypoint mission YAML file."""
    with open(path, encoding='utf-8') as file:
        data = yaml.safe_load(file) or {}

    if not isinstance(data, dict):
        raise ValueError('Mission YAML root must be a mapping')

    frame_id = data.get('frame_id', 'map')
    if frame_id != 'map':
        raise ValueError(
            f'Unsupported mission frame "{frame_id}"; expected "map"'
        )

    raw_waypoints = data.get('waypoints')
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError('Mission must contain at least one waypoint')

    waypoints = []

    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise ValueError(
                f'Waypoint {index} must contain exactly [x, y, z]'
            )

        try:
            x, y, z = (float(value) for value in raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'Waypoint {index} contains a non-numeric value'
            ) from exc

        if not all(math.isfinite(value) for value in (x, y, z)):
            raise ValueError(
                f'Waypoint {index} contains a non-finite value'
            )

        waypoints.append(Waypoint(x=x, y=y, z=z))

    return waypoints


class MissionStateMachine:
    """Manage the lifecycle of a waypoint mission."""

    def __init__(
        self,
        waypoints,
        tolerance,
        timeout,
        takeoff_height=None,
    ):
        if not waypoints:
            raise ValueError('At least one waypoint is required')

        if tolerance <= 0.0:
            raise ValueError('Waypoint tolerance must be positive')

        if timeout <= 0.0:
            raise ValueError('Waypoint timeout must be positive')

        self.waypoints = list(waypoints)
        self.tolerance = float(tolerance)
        self.timeout = float(timeout)

        if takeoff_height is None:
            takeoff_height = self.waypoints[0].z

        self.takeoff_height = float(takeoff_height)

        self.state = IDLE
        self.index = 0
        self.reason = ''

        self._state_before_pause = None
        self._waypoint_started_at = None

    @property
    def current_waypoint(self):
        """Return the active waypoint, if any."""
        if 0 <= self.index < len(self.waypoints):
            return self.waypoints[self.index]

        return None

    def start(self, now):
        """Start or restart the mission."""
        if self.state not in (IDLE, COMPLETED, ABORTED):
            return False, 'Mission is already active'

        self.state = PREFLIGHT
        self.index = 0
        self.reason = ''
        self._state_before_pause = None
        self._waypoint_started_at = None

        return True, 'Mission started'

    def step(
        self,
        now,
        position,
        health,
        safety_state,
        planner_status,
    ):
        """Advance the mission state machine by one control cycle.

        Returned actions are:
          ('none',)
          ('takeoff',)
          ('goal', Waypoint)
          ('land',)

        planner_status is intentionally not used by the direct-route MVP.
        It remains part of the interface for later planner integration.
        """
        now = float(now)
        x, y, z = (float(value) for value in position)

        del x
        del y
        del planner_status

        if self.state in (IDLE, COMPLETED, ABORTED):
            return ('none',)

        if safety_state in (SAFETY_LAND, SAFETY_ABORT):
            reason = (
                'safety_land'
                if safety_state == SAFETY_LAND
                else 'safety_abort'
            )
            self.abort(reason)
            return ('none',)

        if self.state == PAUSED:
            return ('none',)

        if self.state == PREFLIGHT:
            if health != HEALTH_GOOD:
                self.reason = 'waiting_for_good_localization'
                return ('none',)

            if safety_state != SAFETY_NORMAL:
                self.reason = 'waiting_for_normal_safety'
                return ('none',)

            self.reason = ''
            self.state = TAKEOFF
            return ('takeoff',)

        if self.state == TAKEOFF:
            if z >= self.takeoff_height - self.tolerance:
                self.state = NAVIGATE
                self.reason = ''
                self._waypoint_started_at = now
                return ('goal', self.current_waypoint)

            return ('none',)

        if self.state == NAVIGATE:
            waypoint = self.current_waypoint

            if waypoint is None:
                self.state = LANDING
                self._waypoint_started_at = None
                return ('land',)

            if (
                self._waypoint_started_at is not None
                and now - self._waypoint_started_at > self.timeout
            ):
                self.abort('waypoint_timeout')
                return ('none',)

            distance = math.sqrt(
                (position[0] - waypoint.x) ** 2
                + (position[1] - waypoint.y) ** 2
                + (position[2] - waypoint.z) ** 2
            )

            if distance <= self.tolerance:
                self.index += 1

                if self.index >= len(self.waypoints):
                    self.state = LANDING
                    self.reason = ''
                    self._waypoint_started_at = None
                    return ('land',)

                self._waypoint_started_at = now
                return ('goal', self.current_waypoint)

            return ('goal', waypoint)

        if self.state == LANDING:
            if z <= self.tolerance:
                self.state = COMPLETED
                self.reason = ''
                return ('none',)

            return ('none',)

        return ('none',)

    def pause(self):
        """Pause an active mission."""
        if self.state in (IDLE, COMPLETED, ABORTED, PAUSED):
            return False, 'Mission cannot be paused in the current state'

        self._state_before_pause = self.state
        self.state = PAUSED
        self.reason = 'paused'

        return True, 'Mission paused'

    def resume(self, now=None):
        """Resume a previously paused mission."""
        if self.state != PAUSED or self._state_before_pause is None:
            return False, 'Mission is not paused'

        self.state = self._state_before_pause
        self._state_before_pause = None
        self.reason = ''

        if self.state == NAVIGATE and now is not None:
            self._waypoint_started_at = float(now)

        return True, 'Mission resumed'

    def abort(self, reason):
        """Abort the mission and store the reason."""
        if self.state in (COMPLETED, ABORTED):
            return False, 'Mission is already terminal'

        self.state = ABORTED
        self.reason = str(reason)
        self._state_before_pause = None
        self._waypoint_started_at = None

        return True, 'Mission aborted'
