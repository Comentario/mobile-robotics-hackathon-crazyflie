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
from cf2_coverage import CoverageMission
from cf2_layers import LayeredMap, validate_heights


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
                 warning=.5, timeout=30, extent=1.5, coverage_area=None,
                 coverage_spacing=.5, survey_seconds=180, survey_heights=None,
                 scan_yaw_rate=0, mapping_radius=.8):
        self.relative_goals = list(goals)
        self.speed, self.height, self.timeout = speed, height, timeout
        self.clearance, self.warning, self.extent = clearance, warning, extent
        self.coverage_area = coverage_area
        self.coverage_spacing, self.survey_seconds = coverage_spacing, survey_seconds
        self.survey_heights = validate_heights(survey_heights or (height,))
        if not math.isclose(height, self.survey_heights[0]):
            raise ValueError('Takeoff height must match the first survey level')
        if not math.isfinite(scan_yaw_rate) or not 0 <= scan_yaw_rate <= 15:
            raise ValueError('Scan rotation must be between 0 and 15 degrees/s')
        self.height_index, self.completed_layers = 0, []
        self.scan_yaw_rate = scan_yaw_rate
        self.height_changing, self.transition_started, self.height_settled_at = False, None, None
        self.z_limits = (.15, max(.6, max(self.survey_heights)+.12))
        self.coverage_cache, self.coverage_at = None, None
        self.mapper = (LayeredMap(self.survey_heights, mapping_radius=mapping_radius) if coverage_area else
                       LiveMap(width=100, resolution=.05))
        self.health = LocalizationHealthMonitor(HealthConfig(.4, .25, .3, 1.0))
        self.filter = RangeFilter(RangeFilterConfig(.02, 4, 3, .5))
        self.supervisor = SafetySupervisor(SupervisorConfig(
            critical_distance=clearance, hover_timeout=2, max_decel=.3))
        self.avoidance = AvoidanceConfig(warning, clearance, 1.5, speed)
        self.follower = FollowerConfig(.2, speed, .3, .25, height)
        self.limiter = RateLimiter(.3)
        self.output_limiter = RateLimiter(.6)
        self.pose, self.raw_ranges, self.ranges = None, {}, {}
        self.pose_time, self.range_time = None, None
        self.origin, self.fence, self.mission = None, None, None
        self.started = self.last_plan = self.last_tick = None
        self.path, self.replans, self.plan_goal = [], 0, None
        self.no_path_since = self.stale_since = None
        self.pose_stamp, self.range_stamp = None, None
        self.plan_failure = ''
        self.measured_yaw_rate = 0.
        self.estimated_velocity = (0., 0.)

    def update(self, row, now):
        self.mapper.update(dict(row, mapping_paused=True) if self.height_changing else row, now=now)
        name, data = row.get('log'), row.get('data', {})
        stamp = row.get('timestamp_ms')
        if name == 'cf2_pose' and stamp != self.pose_stamp:
            keys = [f'stateEstimate.{k}' for k in ('x', 'y', 'z', 'yaw')]
            if all(k in data and math.isfinite(data[k]) for k in keys):
                if self.pose is not None and now > self.pose_time:
                    dt = now-self.pose_time
                    angle = (data['stateEstimate.yaw']-self.pose['stateEstimate.yaw']+180) % 360-180
                    self.measured_yaw_rate = angle/dt
                    self.estimated_velocity = tuple((data[k]-self.pose[k])/dt for k in keys[:2])
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
        ex, ey = (tuple(v/2 for v in self.coverage_area) if self.coverage_area
                  else (self.extent, self.extent))
        self.fence = Geofence(x-ex, y-ey, self.z_limits[0], x+ex, y+ey, self.z_limits[1])
        if self.coverage_area:
            self.mapper.area, self.mapper.start = tuple(self.coverage_area), self.origin
            # Keep sweep targets outside the reactive wall-warning zone.
            self.mission = CoverageMission(self.origin, self.coverage_area, self.height,
                self.coverage_spacing, min(self.warning+.1, min(self.coverage_area)/2-.2),
                self.timeout, self.survey_seconds/len(self.survey_heights))
            self.mission.start(now)
        elif self.relative_goals:
            goals = [Waypoint(x+dx, y+dy, self.height) for dx, dy in self.relative_goals]
            self.mission = MissionStateMachine(goals, .08, self.timeout, self.height)
            self.mission.start(now)

    def _plan(self, position, goal, now):
        s = (self.mapper.planning_snapshot(now) if self.coverage_area else self.mapper.snapshot(now))
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
        self.replans += 1
        start = world_to_cell(*position[:2], ox, oy, res, w, w)
        end = world_to_cell(goal.x, goal.y, ox, oy, res, w, w)
        if start is None or end is None:
            self.plan_failure = 'outside_planner_grid'
            return []
        if math.isinf(cost[end[1]][end[0]]):
            self.plan_failure = 'goal_inside_obstacle_margin'
            return []
        if math.isinf(cost[start[1]][start[0]]):
            self.plan_failure = 'start_inside_obstacle_margin'
            return []
        cells = astar(cost, start, end, timeout=.08 if self.coverage_area else .04) if start and end else None
        if not cells:
            self.plan_failure = 'planner_search_unresolved'
            return []
        self.plan_failure = ''
        points = [(*cell_to_world(c, ox, oy, res), self.height) for c in cells]
        points[0], points[-1] = position, (goal.x, goal.y, goal.z)
        return densify(remove_collinear(points), .15)

    def tick(self, now):
        result = {'velocity_body': (0., 0., 0.), 'land': False,
                  'state': 'HOVER', 'reason': 'waiting_for_telemetry', 'path': [],
                  'replans': self.replans, 'waypoint_index': 0, 'start': self.origin,
                  'yaw_rate_deg_s': 0., 'survey_heights': list(self.survey_heights),
                  'height_index': self.height_index, 'completed_layers': self.completed_layers}
        if self.coverage_area:
            result['mapping_radius_m'] = self.mapper.mapping_radius
        stale = (self.pose_time is None or self.range_time is None or
                 now-self.pose_time > .4 or now-self.range_time > .4)
        if stale:
            self.output_limiter.reset()
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
        if abs(self.measured_yaw_rate) > 2:
            # Rotating rays sample different surfaces. Their temporal derivative
            # is not object approach speed; use projected translation instead.
            c, s = math.cos(yaw), math.sin(yaw)
            vx, vy = self.estimated_velocity
            bx, by = c*vx+s*vy, -s*vx+c*vy
            rates = {'front': max(0., bx), 'back': max(0., -bx),
                     'left': max(0., by), 'right': max(0., -by)}
        clearances = [effective_clearance(ranges.get(k, math.nan), rates[k], self.supervisor.config)
                      for k in rates]
        ttc = min(time_to_collision(ranges.get(k, math.nan), rates[k]) for k in rates)
        if self.height_changing:
            result.update(start=self.origin, state='CHANGE_HEIGHT',
                          reason='settling_at_next_height', coverage=self.coverage_cache)
            state, scale, reason = self.supervisor.evaluate(SafetyInputs(
                now, health, False, False, False, min(clearances), ttc, pos[2], False,
                not self.fence.contains(pos)))
            error = self.height-pos[2]
            if (pos[2] > self.z_limits[1] or ranges.get('up', math.inf) < .2 or
                    (error > .035 and ranges.get('up', math.nan) < error+.25)):
                result.update(land=True, reason='height_transition_clearance')
                return result
            if now-self.transition_started > 20 or now-self.started >= self.survey_seconds:
                result.update(land=True, reason='height_transition_timeout')
                return result
            if state in (LAND, ABORT):
                result.update(land=True, reason=reason)
                return result
            if (health != 0 or state == 2 or math.isnan(ranges.get('up', math.nan)) or
                    not math.isfinite(ranges.get('down', math.nan))):
                self.height_settled_at = None
                result.update(reason=reason or 'height_sensors_unavailable')
                return result
            if abs(error) <= .025:
                self.height_settled_at = now if self.height_settled_at is None else self.height_settled_at
                if now-self.height_settled_at >= 1:
                    remaining = self.survey_seconds-(now-self.started)
                    self.mission = CoverageMission(self.origin, self.coverage_area, self.height,
                        self.coverage_spacing, min(self.warning+.1, min(self.coverage_area)/2-.2),
                        self.timeout, max(1, remaining/(len(self.survey_heights)-self.height_index)))
                    self.mission.start(now)
                    self.height_changing = False
                    self.path, self.last_plan, self.plan_goal = [], None, None
                    self.coverage_at = None
                    result.update(state='HOVER', reason='height_layer_ready')
                return result
            self.height_settled_at = None
            vz = max(-.08, min(.08, error*.6))
            safe = self.supervisor.filter_command({'mode': MODE_VELOCITY, 'position': pos,
                'velocity': (0., 0., vz), 'yaw': yaw}, state, scale, pos, self.fence, self.z_limits)
            result['velocity_body'] = safe['velocity'] if safe else (0., 0., 0.)
            return result
        goal = self.mission.current_waypoint if self.mission else None
        route_hit = any(self.mapper.scores[cell] > 0 for p in self.path
                        if (cell := self.mapper.cell(*p[:2])) is not None) if self.path else False
        if goal and (self.last_plan is None or now-self.last_plan >= 1 or self.plan_goal != goal or
                     (route_hit and now-self.last_plan >= .2)):
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
            if self.coverage_area and self.mission.phase == 'RETURN' and not self.mission.repair_used:
                self.mission.prepare_repair(pos, now, self.mapper.snapshot(now))
            result['waypoint_index'] = self.mission.index
            if self.coverage_area:
                if self.coverage_at is None or now-self.coverage_at >= .5 or self.mission.state in (LANDING, ABORTED):
                    self.coverage_cache = self.mission.summary(self.mapper.snapshot(now))
                    self.coverage_at = now
                result['coverage'] = self.coverage_cache
            if self.mission.state in (LANDING, ABORTED):
                if self.coverage_area and self.mission.state == LANDING:
                    self.completed_layers.append({'height_m': self.height, 'reason': self.mission.reason,
                                                  'coverage': self.mission.summary(self.mapper.snapshot(now))})
                    if (self.height_index+1 < len(self.survey_heights) and
                            self.mission.reason in ('survey_route_complete', 'survey_partial_returned') and
                            now-self.started < self.survey_seconds-10 and health == 0 and state in (0, 1)):
                        self.height_index += 1
                        self.height = self.survey_heights[self.height_index]
                        self.height_changing, self.transition_started = True, now
                        self.height_settled_at = None
                        self.path, self.last_plan = [], None
                        self.limiter.reset()
                        self.output_limiter.reset()
                        result.update(state='CHANGE_HEIGHT', reason='next_height_from_home',
                                      height_index=self.height_index)
                        return result
                final_reason = self.mission.reason or 'mission_complete'
                if (self.coverage_area and len(self.survey_heights) > 1 and
                        final_reason in ('survey_route_complete', 'survey_partial_returned')):
                    partial = (len(self.completed_layers) < len(self.survey_heights) or
                               any(layer['reason'] != 'survey_route_complete' for layer in self.completed_layers))
                    final_reason = 'survey_levels_partial_returned' if partial else 'survey_levels_returned'
                result.update(land=True, reason=final_reason)
                return result
            if goal != self.mission.current_waypoint:
                self.path, self.last_plan, self.no_path_since = [], None, None
                if not self.coverage_area:
                    self.limiter.reset()
                    self.output_limiter.reset()
                    result.update(reason='next_waypoint')
                    return result
                goal = self.mission.current_waypoint
                if goal:
                    self.path = self._plan(pos, goal, now)
                    self.last_plan, self.plan_goal = now, goal
                no_path = bool(goal and not self.path)
        if not self.coverage_area and now-self.started >= self.timeout * max(1, len(self.relative_goals)):
            result.update(land=True, reason='mission_time_limit')
            return result
        if self.no_path_since is not None and now-self.no_path_since > 5:
            if self.coverage_area:
                self.mission.skip(now, self.plan_failure or 'no_path_in_current_map')
                self.path, self.last_plan, self.no_path_since = [], None, None
                self.limiter.reset()
                self.output_limiter.reset()
                result.update(land=self.mission.state == LANDING,
                              reason=self.mission.reason or 'deferred_blocked_target')
                result['coverage'] = self.mission.summary(self.mapper.snapshot(now))
                return result
            result.update(land=True, reason='no_path_timeout')
            return result
        active_speed = navigation_speed(health, self.speed, self.speed/2)
        requested = (0., 0., 0.)
        target = None
        if goal and self.path and active_speed:
            target = lookahead_point(self.path, pos, .2)
            distance = math.hypot(target[0]-pos[0], target[1]-pos[1])
            if distance > .5:
                target = (pos[0]+(target[0]-pos[0])*.5/distance,
                          pos[1]+(target[1]-pos[1])*.5/distance, self.height)
            requested = desired_velocity(pos, target, active_speed,
                max(.3, math.hypot(goal.x-pos[0], goal.y-pos[1]))
                if self.coverage_area and self.mission.phase != 'RETURN'
                else math.hypot(goal.x-pos[0], goal.y-pos[1]), self.follower)
        dt = .05 if self.last_tick is None else min(.1, max(.001, now-self.last_tick))
        self.last_tick = now
        vx, vy, _ = self.limiter.apply(dt, requested)
        c, s = math.cos(yaw), math.sin(yaw)
        body, blocked, avoidance_reason = reactive_correction(ranges,
            # TTC already scales/blocks via supervisor; do not zero an outward retreat.
            {},
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
        safe = self.supervisor.filter_command(candidate, state, scale, pos, self.fence, self.z_limits)
        retreat = False
        if reason == 'obstacle_close' and health == 0 and self.fence.contains(pos):
            # Close-object HOVER must not cancel a strictly outward escape.
            # Reject motion toward every nearby/invalid destination channel.
            outward, _, _ = reactive_correction(ranges, {}, (0., 0., 0.), self.avoidance)
            ex, ey = outward[:2]
            norm = math.hypot(ex, ey)
            if norm > self.speed:
                ex, ey = ex*self.speed/norm, ey*self.speed/norm
            for k, value in [('front', ex), ('back', -ex), ('left', ey), ('right', -ey)]:
                distance = ranges.get(k, math.nan)
                if value > 0 and (math.isnan(distance) or distance < self.warning):
                    if k in ('front', 'back'): ex = 0.
                    else: ey = 0.
            # Only keep components escaping a threat on the opposite side.
            if not ((ex < 0 and ranges.get('front', math.inf) < self.warning) or
                    (ex > 0 and ranges.get('back', math.inf) < self.warning)):
                ex = 0.
            if not ((ey < 0 and ranges.get('left', math.inf) < self.warning) or
                    (ey > 0 and ranges.get('right', math.inf) < self.warning)):
                ey = 0.
            if ex or ey:
                escape = dict(candidate, velocity=(c*ex-s*ey, s*ex+c*ey, 0))
                safe = self.supervisor.filter_command(escape, 0, 1., pos, self.fence, self.z_limits)
                retreat = True
        if safe is None:
            self.output_limiter.reset()
            result.update(land=state in (LAND, ABORT), reason=reason)
            return result
        vx, vy, _ = safe['velocity']
        if state == 2 and not retreat:
            self.output_limiter.reset()
        else:
            vx, vy, _ = self.output_limiter.apply(dt, (vx, vy, 0.))
        # Acceleration smoothing may never retain motion into a close/invalid direction.
        bx, by = c*vx+s*vy, -s*vx+c*vy
        for k, value in [('front', bx), ('back', -bx), ('left', by), ('right', -by)]:
            distance = ranges.get(k, math.nan)
            threshold = self.warning if retreat else self.clearance
            if value > 0 and (math.isnan(distance) or distance < threshold):
                if k in ('front', 'back'): bx = 0.
                else: by = 0.
        projected = self.supervisor.filter_command(
            dict(candidate, velocity=(c*bx-s*by, s*bx+c*by, 0.)),
            0, 1., pos, self.fence, self.z_limits)
        vx, vy, _ = projected['velocity']
        self.output_limiter.reset((vx, vy, 0.))
        # Vertical range/height problems request the normal landing routine, never a motor cut.
        vertical_problem = pos[2] > self.z_limits[1] or ranges.get('up', math.inf) < .2
        result.update(velocity_body=(c*vx+s*vy, -s*vx+c*vy, 0.),
            land=vertical_problem, state='RETREAT' if retreat else ('NORMAL', 'SLOW', 'HOVER', 'LAND', 'ABORT')[state],
            reason='vertical_envelope' if vertical_problem else (reason or avoidance_reason or health_reason),
            path=self.path, replans=self.replans, start=self.origin,
            planner_failure=self.plan_failure,
            yaw_rate_deg_s=self.scan_yaw_rate if (self.coverage_area and not retreat and
                state in (0,1) and health == 0 and min(clearances) >= self.clearance and
                not vertical_problem and self.mission.phase != 'RETURN') else 0.,
            local_target=target,
            workflow='RETREAT' if retreat else ('RETURN' if self.coverage_area and self.mission.phase == 'RETURN' else
                     ('MOVE' if state in (0, 1) else 'HOLD')),
            relative_xy=(pos[0]-self.origin[0], pos[1]-self.origin[1]))
        return result
