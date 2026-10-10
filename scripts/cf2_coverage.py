"""Bounded online coverage schedule; positions are in the local EKF frame.

Visited survey targets are NOT equivalent to measured map coverage. Blocked
targets are revisited after surveying elsewhere; a failure is reported as
unresolved, never as proof of physical impossibility.
"""
import math

from mission_manager.mission import Waypoint, NAVIGATE, LANDING, ABORTED


class CoverageMission:
    def __init__(self, origin, area, height, spacing=.5, margin=.35,
                 timeout=30, budget=180):
        self.origin, self.area = tuple(origin), tuple(area)
        self.height, self.timeout, self.budget = height, timeout, budget
        self.margin, self.repair_used = margin, False
        width, length = area
        def axis(size):
            count = math.ceil((size-2*margin)/spacing)
            return [-size/2+margin+i*(size-2*margin)/count for i in range(count+1)]
        xs, ys = axis(width), axis(length)
        # Endpoints define continuous sweeps; dense intermediate goals would
        # unnecessarily brake the vehicle every few centimetres. A perpendicular
        # pass improves observation of areas between the four narrow ToF rays.
        self.targets = [(origin[0]+x, origin[1]+y) for i, y in enumerate(ys)
                        for x in ((xs[0], xs[-1]) if i % 2 == 0 else (xs[-1], xs[0]))]
        self.cross_start = len(self.targets)
        cross_xs = xs if self.targets[-1][0] == origin[0]+xs[0] else xs[::-1]
        for i, x in enumerate(cross_xs):
            for y in ((ys[-1], ys[0]) if i % 2 == 0 else (ys[0], ys[-1])):
                xy = (origin[0]+x, origin[1]+y)
                if xy != self.targets[-1]:
                    self.targets.append(xy)
        # Keep successive sweep targets nearby, including the approach from
        # takeoff. The follower preserves velocity between reached targets.
        endpoints, self.targets = self.targets, []
        cursor = self.origin
        old_cross = self.cross_start
        for index, endpoint in enumerate(endpoints):
            if index == old_cross:
                self.cross_start = len(self.targets)
            steps = max(1, math.ceil(math.dist(cursor, endpoint)/spacing))
            self.targets.extend((cursor[0]+(endpoint[0]-cursor[0])*j/steps,
                                 cursor[1]+(endpoint[1]-cursor[1])*j/steps)
                                for j in range(1, steps+1))
            cursor = endpoint
        self.pending = list(range(len(self.targets)))
        self.visited, self.deferred, self.unresolved = [], [], []
        self.attempts = {}
        self.active = None
        self.phase, self.state, self.reason = 'SURVEY', NAVIGATE, ''
        self.started = self.goal_started = None
        self.index = 0

    @property
    def current_waypoint(self):
        xy = self.origin if self.phase == 'RETURN' else (
            self.targets[self.active] if self.active is not None else None)
        return Waypoint(*xy, self.height) if xy is not None else None

    def start(self, now):
        self.started = now
        self._next(now)

    def _next(self, now):
        if not self.pending and self.deferred:
            self.pending, self.deferred = self.deferred, []
            self.phase = 'REVISIT'
        if self.pending:
            self.active = self.pending.pop(0)
            self.attempts[self.active] = self.attempts.get(self.active, 0)+1
            if not self.repair_used and self.attempts[self.active] == 1 and self.active >= self.cross_start:
                self.phase = 'CROSS_SURVEY'
        else:
            self.active, self.phase = None, 'RETURN'
        self.goal_started = now
        self.index = len(self.visited)

    def skip(self, now, reason):
        if self.phase == 'RETURN':
            self.state, self.reason = LANDING, 'return_unresolved:'+reason
            return
        target = self.active
        if self.attempts[target] < 2:
            self.deferred.append(target)
        else:
            self.unresolved.append({'target': self.targets[target], 'reason': reason})
        self._next(now)

    def step(self, now, position, health, safety_state, planner_status):
        if safety_state in (3, 4):
            self.state, self.reason = ABORTED, 'safety_land'
            return
        if self.phase != 'RETURN' and now-self.started >= self.budget:
            self.unresolved += [{'target': self.targets[i], 'reason': 'survey_budget'}
                                for i in [self.active, *self.pending, *self.deferred]
                                if i is not None]
            self.pending, self.deferred, self.active = [], [], None
            self.phase, self.goal_started = 'RETURN', now
        goal = self.current_waypoint
        if math.dist(position, (goal.x, goal.y, goal.z)) <= .10:
            if self.phase == 'RETURN':
                self.state = LANDING
                self.reason = 'survey_partial_returned' if self.unresolved else 'survey_route_complete'
            else:
                self.visited.append(self.active)
                self._next(now)
        elif now-self.goal_started > self.timeout:
            self.skip(now, 'progress_timeout')

    def prepare_repair(self, position, now, snapshot):
        """One bounded extra pass to unknown sectors adjacent to observed free space."""
        if (self.phase != 'RETURN' or self.state != NAVIGATE or self.repair_used or
                now-self.started >= self.budget-10 or snapshot['origin'] is None):
            return
        self.repair_used = True
        mx, my = snapshot['origin']
        res, width, grid = snapshot['resolution'], snapshot['width'], snapshot['data']
        stride = max(1, round(.3/res))
        candidates = []
        for row in range(stride, width-stride, stride):
            for col in range(stride, width-stride, stride):
                x, y = mx+(col+.5)*res, my+(row+.5)*res
                if (abs(x-self.origin[0]) > self.area[0]/2-self.margin or
                        abs(y-self.origin[1]) > self.area[1]/2-self.margin):
                    continue
                if grid[row*width+col] != -1:
                    continue
                # Inspect the full patch: four sampled neighbours can miss a
                # narrow measured ray purely because of grid alignment.
                if any(grid[(row+dy)*width+col+dx] == 0
                       for dy in range(-stride, stride+1)
                       for dx in range(-stride, stride+1)):
                    candidates.append((x, y))
        cursor = position[:2]
        # Near-to-far ordering with spatial separation avoids revisiting the
        # same small unknown patch repeatedly; A* still checks each target.
        for _ in range(12):
            if not candidates:
                break
            target = min(candidates, key=lambda p: math.dist(p, cursor))
            candidates = [p for p in candidates if math.dist(p, target) > .35]
            self.pending.append(len(self.targets))
            self.targets.append(target)
            cursor = target
        if self.pending:
            self.phase = 'REPAIR_SURVEY'
            self._next(now)

    def summary(self, snapshot):
        width, length = self.area
        ox, oy = self.origin
        observed = total = occupied = 0
        mx, my = snapshot['origin'] or (ox, oy)
        res, count = snapshot['resolution'], snapshot['width']
        for i, value in enumerate(snapshot['data']):
            x, y = mx+(i % count+.5)*res, my+(i // count+.5)*res
            if abs(x-ox) <= width/2 and abs(y-oy) <= length/2:
                total += 1
                observed += value >= 0
                occupied += value >= 50
        return {'area': list(self.area), 'phase': self.phase,
                'targets': self.targets, 'visited': self.visited,
                'active_target': self.active, 'unresolved': self.unresolved,
                'deferred_count': len(self.deferred),
                'observed_percent': round(100*observed/total, 1) if total else 0.,
                'unknown_cells': total-observed,
                'occupied_cells': occupied, 'survey_budget_seconds': self.budget,
                'proof_of_impossibility': False}
