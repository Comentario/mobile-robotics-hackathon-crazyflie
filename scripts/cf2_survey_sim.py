"""Live RViz mapping simulation: fixed sweep, static boxes, no radio imports.

Ideal pose and four scalar rays. World geometry is used ONLY by sensor generation
and collision evaluation, never to initialize occupancy or choose a detour.
"""
import argparse
import json
import math
import socket
import time
from pathlib import Path
from contextlib import ExitStack
from uuid import uuid4

from cf2_navigation import (astar, build_cost_map, world_to_cell, cell_to_world,
                            reactive_correction, AvoidanceConfig)
from cf2_layers import LayeredMap
from cf2_coverage import CoverageMission
from cf2_build_map import ANGLES


def scene(name):
    # xmin, ymin, xmax, ymax, top height. Panels intersect the 0.30 m plane.
    return [] if name == 'empty' else [(-.8, -.55, -.45, .25, .65),
                                     (.65, .25, 1.0, .7, .7),
                                     (1.35, -.85, 1.65, -.55, .15)]


def ray_box(x, y, angle, box):
    dx, dy = math.cos(angle), math.sin(angle)
    low, high = -math.inf, math.inf
    for p, v, lower, upper in ((x, dx, box[0], box[2]), (y, dy, box[1], box[3])):
        if abs(v) < 1e-9:
            if not lower <= p <= upper:
                return math.inf
        else:
            a, b = sorted(((lower-p)/v, (upper-p)/v))
            low, high = max(low, a), min(high, b)
    return max(0., low) if high >= max(0., low) else math.inf


def ranges(x, y, yaw, area, boxes, height):
    values = {}
    for name, offset in ANGLES.items():
        angle = yaw+offset
        dx, dy = math.cos(angle), math.sin(angle)
        walls = []
        if abs(dx) > 1e-9:
            walls.append(((area[0]/2 if dx > 0 else -area[0]/2)-x)/dx)
        if abs(dy) > 1e-9:
            walls.append(((area[1]/2 if dy > 0 else -area[1]/2)-y)/dy)
        distance = min(walls + [ray_box(x, y, angle, b) for b in boxes if b[4] >= height])
        values['range.'+name] = round(distance*1000) if 0 <= distance <= 4 else 32766
    values.update({'range.up': 1500, 'range.zrange': round(height*1000)})
    return values


def sweep(area, spacing, margin=.35):
    left, right = -area[0]/2+margin, area[0]/2-margin
    bottom, top = -area[1]/2+margin, area[1]/2-margin
    lanes = max(1, math.ceil((right-left)/spacing))
    points = []
    for lane in range(lanes+1):
        x = left+(right-left)*lane/lanes
        points.extend([(x, top), (x, bottom)] if lane % 2 == 0 else [(x, bottom), (x, top)])
    return points


class SurveySimulation:
    def __init__(self, area=(5., 3.), spacing=.35, speed=.15, yaw_rate=0.,
                 reveal=.3, scenario='empty', height=.3):
        self.area, self.speed, self.yaw_rate, self.height = area, speed, yaw_rate, height
        self.boxes = scene(scenario)
        self.mapper = LayeredMap((height,), free_radius=reveal, mapping_radius=1.2)
        self.mapper.area, self.mapper.start = tuple(area), (0., 0.)
        self.session = 'simulation_'+uuid4().hex[:8]
        self.mission = CoverageMission((0., 0.), area, height)
        self.mission.targets = sweep(area, spacing)
        self.targets = self.mission.targets
        self.index = 0
        self.x = self.y = self.yaw = self.elapsed = self.stuck = 0.
        self.path = []
        self.last_plan = -math.inf
        self.done, self.reason = False, ''
        self.detours = 0
        self.progress_at = 0.
        self.best_distance = math.inf
        self.reveal = reveal
        self.avoidance = AvoidanceConfig(.45, .25, 1.5, speed)

    def blocked(self, start, end, snapshot):
        ox, oy = snapshot['origin']
        w, res = snapshot['width'], snapshot['resolution']
        distance = math.dist(start, end)
        dx, dy = end[0]-start[0], end[1]-start[1]
        for i, value in enumerate(snapshot['data']):
            if value < 50:
                continue
            p = (ox+(i%w+.5)*res, oy+(i//w+.5)*res)
            t = max(0., min(1., ((p[0]-start[0])*dx+(p[1]-start[1])*dy)/(distance**2 or 1)))
            if math.dist(p, (start[0]+t*dx, start[1]+t*dy)) < .28:
                return True
        return False

    def detour(self, goal, snapshot):
        w, res, (ox, oy) = snapshot['width'], snapshot['resolution'], snapshot['origin']
        grid = [snapshot['data'][y*w:(y+1)*w] for y in range(w)]
        inflated = [row[:] for row in grid]
        radius = math.ceil(.25/res)
        for y in range(w):
            for x in range(w):
                if grid[y][x] >= 50:
                    for dy in range(-radius, radius+1):
                        for dx in range(-radius, radius+1):
                            if dx*dx+dy*dy <= radius*radius and 0<=x+dx<w and 0<=y+dy<w:
                                inflated[y+dy][x+dx] = 100
        for y in range(w):
            for x in range(w):
                px, py = cell_to_world((x, y), ox, oy, res)
                if abs(px)>self.area[0]/2-.2 or abs(py)>self.area[1]/2-.2:
                    inflated[y][x] = 100
        cost = build_cost_map(inflated, 'penalize', 2, 0)
        start = world_to_cell(self.x, self.y, ox, oy, res, w, w)
        end = world_to_cell(*goal, ox, oy, res, w, w)
        path = astar(cost, start, end, timeout=.15)
        self.last_plan = self.elapsed
        self.detours += 1
        return [cell_to_world(p, ox, oy, res) for p in path[1:]] if path else []

    def step(self, dt=.1):
        self.elapsed += dt
        readings = ranges(self.x, self.y, self.yaw, self.area, self.boxes, self.height)
        common = dict(session_id=self.session, timestamp_ms=round(self.elapsed*1000), hovering=True,
                      phase='simulation')
        rows = [dict(common, log='cf2_navigation', data=dict(simulation=True, source='IDEAL SIMULATION', start=[0, 0], survey_heights=[self.height],
                 mapping_radius_m=1.2, free_radius_m=self.reveal, coverage={'area':self.area})),
                dict(common, log='cf2_pose', data={'stateEstimate.x':self.x, 'stateEstimate.y':self.y,
                     'stateEstimate.z':self.height, 'stateEstimate.yaw':math.degrees(self.yaw)}),
                dict(common, log='cf2_ranges', data=readings)]
        for row in rows:
            self.mapper.update(row, now=self.elapsed)
        snapshot = self.mapper.snapshot(self.elapsed)
        goal = self.targets[self.index] if self.index<len(self.targets) else (0., 0.)
        goal_distance = math.dist((self.x,self.y), goal)
        if goal_distance < self.best_distance-.03:
            self.best_distance, self.progress_at = goal_distance, self.elapsed
        if goal_distance<.07:
            if self.index<len(self.targets):
                self.mission.visited.append(self.index)
                self.index += 1
                self.path = []
                self.stuck = 0.
                self.best_distance, self.progress_at = math.inf, self.elapsed
            else:
                self.done, self.reason = True, 'sweep_returned'
        target = goal
        direct_blocked = self.blocked((self.x,self.y), goal, snapshot)
        if direct_blocked and self.elapsed-self.last_plan>=1:
            self.path = self.detour(goal, snapshot)
        while self.path and math.dist((self.x,self.y),self.path[0])<.07:
            self.path.pop(0)
        if self.path:
            target = self.path[0]
        dx, dy = target[0]-self.x, target[1]-self.y
        norm = math.hypot(dx,dy)
        factor = min(self.speed, norm*1.5)/(norm or 1.)
        vx, vy = dx*factor, dy*factor
        if direct_blocked and not self.path:
            vx = vy = 0.
        c,s = math.cos(self.yaw), math.sin(self.yaw)
        measured = {name: readings['range.'+name]/1000 if readings['range.'+name]<4000
                    else math.inf for name in ANGLES}
        body, _, _ = reactive_correction(measured, {}, (c*vx+s*vy,-s*vx+c*vy,0), self.avoidance)
        vx,vy = c*body[0]-s*body[1], s*body[0]+c*body[1]
        self.stuck = self.stuck+dt if math.hypot(vx,vy)<.015 else 0.
        if self.stuck>12 or self.elapsed-self.progress_at>25:
            if self.index<len(self.targets):
                self.mission.unresolved.append({'target':goal,'reason':'blocked_in_observed_map'})
                self.index += 1
                self.path = []
                self.stuck = 0.
                self.best_distance, self.progress_at = math.inf, self.elapsed
            else:
                self.done, self.reason = True, 'return_unresolved'
        self.mission.phase = 'SWEEP' if self.index<len(self.targets) else 'RETURN'
        self.mission.active = self.index if self.index<len(self.targets) else None
        coverage = self.mission.summary(snapshot)
        coverage.update(mode='fixed_simulation', finish_trigger=self.reason)
        decision = dict(simulation=True, source='IDEAL SIMULATION', state='LAND' if self.done else 'NORMAL',
            workflow=self.mission.phase, reason=self.reason, start=[0,0], path=[(*p,self.height) for p in self.path],
            local_target=(*target,self.height), coverage=coverage, free_radius_m=self.reveal,
            mapping_radius_m=1.2, survey_heights=[self.height], detours=self.detours)
        rows.append(dict(common, log='cf2_navigation', data=decision))
        self.mapper.update(rows[-1], now=self.elapsed)
        if not self.done:
            nx,ny = self.x+vx*dt, self.y+vy*dt
            # Evaluation only: a collision fails the simulation, not a hidden avoidance aid.
            collision = any(b[4]>=self.height and b[0]-.12<nx<b[2]+.12 and b[1]-.12<ny<b[3]+.12
                            for b in self.boxes)
            if collision:
                raise RuntimeError('SIMULATION COLLISION: controller failed; no successful map claimed')
            self.x,self.y = nx,ny
            self.yaw += math.radians(self.yaw_rate)*dt
        return rows, decision


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('empty','obstacles'), default='empty')
    parser.add_argument('--yaw-rate', type=float, default=0, help='degrees/s; start with 0')
    parser.add_argument('--reveal-radius', type=float, default=.3)
    parser.add_argument('--spacing', type=float, default=.35)
    parser.add_argument('--speed', type=float, default=.15)
    parser.add_argument('--rate', type=float, default=4, help='Playback speed, not flight speed')
    parser.add_argument('--seconds', type=float, default=600, help='Simulation budget')
    parser.add_argument('--udp-port', type=int, default=8766)
    parser.add_argument('--headless', action='store_true', help='No UDP, run as fast as possible')
    args=parser.parse_args()
    for value,lower,upper in [(args.reveal_radius,.1,.4),(args.spacing,.2,.6),(args.speed,.05,.2),
                              (args.rate,.1,20),(args.seconds,1,3600),(args.yaw_rate,0,15)]:
        if not math.isfinite(value) or not lower<=value<=upper:
            parser.error('Invalid simulation parameter')
    sim=SurveySimulation(spacing=args.spacing,speed=args.speed,yaw_rate=args.yaw_rate,
                         reveal=args.reveal_radius,scenario=args.scenario)
    print('IDEAL SIMULATION ONLY | No Crazyradio or flight commands | '+args.scenario, flush=True)
    with ExitStack() as stack:
        udp = None if args.headless else stack.enter_context(socket.socket(socket.AF_INET,socket.SOCK_DGRAM))
        try:
            while sim.elapsed<args.seconds and not sim.done:
                start=time.monotonic()
                rows,decision=sim.step()
                if not args.headless:
                    for row in rows: udp.sendto(json.dumps(row).encode(),('127.0.0.1',args.udp_port))
                    time.sleep(max(0,.1/args.rate-(time.monotonic()-start)))
            if not sim.done: sim.reason='simulation_budget_partial'
        except KeyboardInterrupt:
            sim.reason='simulation_interrupted_partial'
        except RuntimeError as error:
            sim.reason='simulation_failed_collision'
            print(str(error), flush=True)
    decision['reason'] = sim.reason
    decision['coverage']['finish_trigger'] = sim.reason
    if sim.reason != 'sweep_returned':
        decision['state'] = 'PARTIAL'
    snapshot=sim.mapper.snapshot(sim.elapsed)
    snapshot.update(simulation=True, navigation=decision, map_validated_for_drone1=False)
    target=Path('data/runs')/f'cf2_sim_{sim.session}_map.json'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(snapshot))
    print(f"{sim.reason} | observed={decision['coverage']['observed_percent']}% | "
          f"visited={len(sim.mission.visited)}/{len(sim.targets)} | detours={sim.detours} | {target}")
    if sim.reason == 'simulation_failed_collision':
        raise SystemExit(1)

if __name__=='__main__':main()
