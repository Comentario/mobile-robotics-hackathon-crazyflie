"""Direct-radio navigation adapter using the team's pure TP algorithms.

No ROS or hardware imports here. Velocity outputs are BODY-frame m/s.
Map coordinates remain in the session EKF frame; goals are relative to start.
"""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for package in ('global_planner', 'path_processor', 'local_navigator',
                'mission_manager', 'range_processor', 'state_monitor', 'safety_supervisor'):
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / package))

from global_planner.astar import astar, build_cost_map, world_to_cell, cell_to_world
from path_processor.simplify import remove_collinear, densify
from local_navigator.follower import FollowerConfig, RateLimiter, desired_velocity, lookahead_point
from local_navigator.avoidance import AvoidanceConfig, reactive_correction
from local_navigator.speed_policy import navigation_speed
from mission_manager.mission import MissionStateMachine, Waypoint, LANDING, ABORTED
from range_processor.range_filter import RangeFilter, RangeFilterConfig, time_to_collision
from state_monitor.health import LocalizationHealthMonitor, HealthConfig
from safety_supervisor.supervisor import (SafetySupervisor, SupervisorConfig, SafetyInputs,
    effective_clearance, MODE_VELOCITY, LAND, ABORT)
from safety_supervisor.geofence import Geofence
from cf2_live_map import LiveMap


def metres(raw):
    if raw is None or not math.isfinite(raw) or raw < 0:
        return math.nan
    # Same operational interpretation as cflib Multiranger: beyond sensor range.
    # The mapper separately leaves these returns UNKNOWN, never free.
    if raw >= 4000:
        return math.inf
    return raw / 1000


class Navigation:
    def __init__(self, goals=(), speed=.1, height=.3, clearance=.25,
                 warning=.5, timeout=30, extent=1.5):
        self.relative_goals = list(goals)
        self.speed, self.height, self.timeout = speed, height, timeout
        self.clearance, self.warning, self.extent = clearance, warning, extent
        self.mapper = LiveMap(width=100, resolution=.05)
        self.health = LocalizationHealthMonitor(HealthConfig(.4, .25, .3, 1.0))
        self.filter = RangeFilter(RangeFilterConfig(.02, 4, 3, .5))
        self.supervisor = SafetySupervisor(SupervisorConfig(
            critical_distance=clearance, hover_timeout=2, max_decel=.3))
        self.avoidance = AvoidanceConfig(warning, clearance, 1.5, speed)
        self.follower = FollowerConfig(.2, speed, .3, .25, height)
        self.limiter = RateLimiter(.3)
        self.pose, self.raw_ranges, self.ranges = None, {}, {}
        self.pose_time, self.range_time = None, None
        self.origin, self.fence, self.mission = None, None, None
        self.started = self.last_plan = self.last_tick = None
        self.path, self.replans, self.plan_goal = [], 0, None
        self.no_path_since = self.stale_since = None
        self.pose_stamp, self.range_stamp = None, None

    def update(self, row, now):
        self.mapper.update(row, now=now)
        name, data = row.get('log'), row.get('data', {})
        stamp = row.get('timestamp_ms')
        if name == 'cf2_pose' and stamp != self.pose_stamp:
            keys = [f'stateEstimate.{k}' for k in ('x', 'y', 'z', 'yaw')]
            if all(k in data and math.isfinite(data[k]) for k in keys):
                self.pose, self.pose_time, self.pose_stamp = dict(data), now, stamp
                self.health.update(now, tuple(data[k] for k in keys[:3]))
        if name == 'cf2_ranges' and stamp != self.range_stamp:
            self.raw_ranges = {k: metres(data.get('range.' + ('zrange' if k == 'down' else k)))
                               for k in ('front', 'back', 'left', 'right', 'up', 'down')}
            self.ranges = self.filter.update(now, self.raw_ranges)
            self.range_time, self.range_stamp = now, stamp

    def _begin(self, now):
        self.origin = tuple(self.pose[f'stateEstimate.{k}'] for k in ('x', 'y'))
        self.started = now
        x, y = self.origin
        self.fence = Geofence(x-self.extent, y-self.extent, .15,
                              x+self.extent, y+self.extent, .6)
        if self.relative_goals:
            goals = [Waypoint(x+dx, y+dy, self.height) for dx, dy in self.relative_goals]
            self.mission = MissionStateMachine(goals, .08, self.timeout, self.height)
            self.mission.start(now)

    def _plan(self, position, goal, now):
        s = self.mapper.snapshot(now)
        if s['origin'] is None:
            return []
        w, res, (ox, oy) = s['width'], s['resolution'], s['origin']
        grid = [s['data'][y*w:(y+1)*w] for y in range(w)]
        inflated = [row[:] for row in grid]
        radius = math.ceil(self.clearance / res)
        offsets = [(dx, dy) for dx in range(-radius, radius+1)
                   for dy in range(-radius, radius+1) if math.hypot(dx, dy)*res <= self.clearance]
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                if value >= 50:
                    for dx, dy in offsets:
                        if 0 <= x+dx < w and 0 <= y+dy < w:
                            inflated[y+dy][x+dx] = 100
        # Exclude outside the local operational envelope from planning.
        for y, row in enumerate(inflated):
            for x in range(w):
                wx, wy = cell_to_world((x, y), ox, oy, res)
                if not self.fence.contains((wx, wy, self.height)):
                    row[x] = 100
        cost = build_cost_map(inflated, 'penalize', 3, 0)
        start = world_to_cell(*position[:2], ox, oy, res, w, w)
        end = world_to_cell(goal.x, goal.y, ox, oy, res, w, w)
        cells = astar(cost, start, end, timeout=.04) if start and end else None
        self.replans += 1
        if not cells:
            return []
        points = [(*cell_to_world(c, ox, oy, res), self.height) for c in cells]
        points[0], points[-1] = position, (goal.x, goal.y, goal.z)
        return densify(remove_collinear(points), .15)

    def tick(self, now):
        result = {'velocity_body': (0., 0., 0.), 'land': False,
                  'state': 'HOVER', 'reason': 'waiting_for_telemetry', 'path': [],
                  'replans': self.replans, 'waypoint_index': 0, 'start': self.origin}
        stale = (self.pose_time is None or self.range_time is None or
                 now-self.pose_time > .4 or now-self.range_time > .4)
        if stale:
            self.stale_since = now if self.stale_since is None else self.stale_since
            result.update(reason='telemetry_stale', land=now-self.stale_since >= 2)
            return result
        self.stale_since = None
        if self.origin is None:
            self._begin(now)
        pos = tuple(self.pose[f'stateEstimate.{k}'] for k in ('x', 'y', 'z'))
        yaw = math.radians(self.pose['stateEstimate.yaw'])
        health, age, health_reason = self.health.evaluate(now)
        ranges = dict(self.ranges)
        # Near raw returns override the median immediately.
        for k, raw in self.raw_ranges.items():
            if raw == -math.inf or math.isfinite(raw):
                ranges[k] = min(raw, ranges.get(k, math.inf)) if not math.isnan(ranges.get(k, math.nan)) else raw
        rates = {k: self.filter.closing_rate(k) for k in ('front', 'back', 'left', 'right')}
        clearances = [effective_clearance(ranges.get(k, math.nan), rates[k], self.supervisor.config)
                      for k in rates]
        ttc = min(time_to_collision(ranges.get(k, math.nan), rates[k]) for k in rates)
        goal = self.mission.current_waypoint if self.mission else None
        if goal and (self.last_plan is None or now-self.last_plan >= 1 or self.plan_goal != goal):
            self.path = self._plan(pos, goal, now)
            self.last_plan, self.plan_goal = now, goal
        no_path = bool(goal and not self.path)
        self.no_path_since = (now if self.no_path_since is None else self.no_path_since) if no_path else None
        state, scale, reason = self.supervisor.evaluate(SafetyInputs(
            now, health, False, False, False, min(clearances), ttc, pos[2], no_path,
            not self.fence.contains(pos)))
        if self.mission:
            # Planner availability is enforced by supervisor; TP-26 owns lifecycle.
            self.mission.step(now, pos, health, state, 0)
            result['waypoint_index'] = self.mission.index
            if self.mission.state in (LANDING, ABORTED):
                result.update(land=True, reason=self.mission.reason or 'mission_complete')
                return result
        if now-self.started >= self.timeout * max(1, len(self.relative_goals)):
            result.update(land=True, reason='mission_time_limit')
            return result
        if self.no_path_since is not None and now-self.no_path_since > 5:
            result.update(land=True, reason='no_path_timeout')
            return result
        active_speed = navigation_speed(health, self.speed, self.speed/2)
        requested = (0., 0., 0.)
        if goal and self.path and active_speed:
            target = lookahead_point(self.path, pos, .2)
            requested = desired_velocity(pos, target, active_speed,
                math.hypot(goal.x-pos[0], goal.y-pos[1]), self.follower)
        dt = .05 if self.last_tick is None else min(.1, max(.001, now-self.last_tick))
        self.last_tick = now
        vx, vy, _ = self.limiter.apply(dt, requested)
        c, s = math.cos(yaw), math.sin(yaw)
        body, blocked, avoidance_reason = reactive_correction(ranges,
            {k: -v if v is not None else math.nan for k, v in rates.items()},
            (c*vx+s*vy, -s*vx+c*vy, 0), self.avoidance)
        norm = math.hypot(*body[:2])
        factor = min(1, self.speed/norm) if norm else 1
        bx, by = body[0]*factor, body[1]*factor
        # No motion into an unavailable channel. No-return is +inf, not NaN.
        for k, value in [('front', bx), ('back', -bx), ('left', by), ('right', -by)]:
            if value > 0 and math.isnan(ranges.get(k, math.nan)):
                if k in ('front', 'back'): bx = 0.
                else: by = 0.
        candidate = {'mode': MODE_VELOCITY, 'position': pos,
                     'velocity': (c*bx-s*by, s*bx+c*by, 0), 'yaw': yaw}
        safe = self.supervisor.filter_command(candidate, state, scale, pos, self.fence, (.15, .6))
        if safe is None:
            result.update(land=state in (LAND, ABORT), reason=reason)
            return result
        vx, vy, _ = safe['velocity']
        # Vertical range/height problems request the normal landing routine, never a motor cut.
        vertical_problem = pos[2] > .6 or ranges.get('up', math.inf) < .2
        result.update(velocity_body=(c*vx+s*vy, -s*vx+c*vy, 0.),
            land=vertical_problem, state=('NORMAL', 'SLOW', 'HOVER', 'LAND', 'ABORT')[state],
            reason='vertical_envelope' if vertical_problem else (reason or avoidance_reason or health_reason),
            path=self.path, replans=self.replans, start=self.origin,
            relative_xy=(pos[0]-self.origin[0], pos[1]-self.origin[1]))
        return result
