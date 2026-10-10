"""Persistent discrete height maps in one session frame, with measured fog of war.

The composite is a visualization only. Planning uses the CURRENT height map;
the export includes separate layers and never asserts free space at every Z.
"""
import math
import time
from threading import RLock
from functools import wraps
from cf2_live_map import LiveMap


def synchronized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return call


def validate_heights(heights):
    values = tuple(float(z) for z in heights)
    if not 1 <= len(values) <= 5 or not all(math.isfinite(z) and .2 <= z <= .8 for z in values):
        raise ValueError('Choose 1 to 5 height levels between 0.2 and 0.8 m')
    if any(b-a < .099999 for a, b in zip(values, values[1:])):
        raise ValueError('Height levels must increase with at least 0.10 m separation')
    return values


class LayeredMap:
    def __init__(self, heights=(.3,), width=160, resolution=.05, tolerance=.045,
                 decay_seconds=600, persistent_seconds=600, retain_observations=True):
        self.heights = validate_heights(heights)
        self.width, self.resolution, self.tolerance = width, resolution, tolerance
        self.decay_seconds, self.persistent_seconds = decay_seconds, persistent_seconds
        self.retain_observations = retain_observations
        self.lock = RLock()
        self.reset('waiting')

    def reset(self, session):
        self.session, self.maps = session, {}
        self.active = 0
        self.pose, self.ranges, self.start, self.navigation = None, {}, None, None
        self.last_pose_at = self.last_ranges_at = None
        self.last_pose_stamp = None
        self.area, self.grid_origin = None, None
        self.trail = []
        self.mask_key, self.mask = None, []
        self._configure(self.heights)

    def _configure(self, heights):
        self.heights = validate_heights(heights)
        for z in self.heights:
            if z not in self.maps:
                self.maps[z] = LiveMap(self.resolution, self.width, z-self.tolerance,
                    z+self.tolerance, decay_seconds=self.decay_seconds,
                    persistent_seconds=self.persistent_seconds, retain_observations=self.retain_observations)
                self.maps[z].reset(self.session)
                self.maps[z].origin = self.grid_origin
                self.maps[z].start = self.start
        self.active = min(self.active, len(self.heights)-1)

    @property
    def current(self):
        return self.maps[self.heights[self.active]]

    @property
    def scores(self):
        return self.current.scores

    def cell(self, x, y):
        return self.current.cell(x, y) if self.current.origin is not None else None

    @synchronized
    def update(self, row, now=None):
        now = time.monotonic() if now is None else now
        session = row.get('session_id', 'default')
        if session != self.session:
            self.reset(session)
        name, data = row.get('log'), row.get('data', {})
        if name == 'cf2_navigation':
            self.navigation = dict(data)
            if data.get('survey_heights'):
                self._configure(data['survey_heights'])
            reference = data.get('start')
            if reference is not None and len(reference) == 2 and all(math.isfinite(v) for v in reference):
                self.start = tuple(reference)
            self.area = tuple(data['coverage']['area']) if data.get('coverage') else self.area
            for m in self.maps.values():
                m.start = self.start
            return
        if name == 'cf2_pose':
            keys = [f'stateEstimate.{a}' for a in ('x', 'y', 'z', 'yaw')]
            if not all(a in data and math.isfinite(data[a]) for a in keys):
                return
            self.pose, self.last_pose_at = dict(data), now
            z = data['stateEstimate.z']
            nearest = min(range(len(self.heights)), key=lambda i: abs(self.heights[i]-z))
            matched = abs(self.heights[nearest]-z) <= self.tolerance
            if matched:
                self.active = nearest
            if row.get('hovering') and matched and self.grid_origin is None:
                x, y = data['stateEstimate.x'], data['stateEstimate.y']
                half = self.width*self.resolution/2
                self.grid_origin = (x-half, y-half)
                self.start = self.start or (x, y)
                for m in self.maps.values():
                    m.origin, m.start = self.grid_origin, self.start
            if row.get('hovering') and row['timestamp_ms'] != self.last_pose_stamp:
                self.trail.append([data[a] for a in keys[:3]])
                self.last_pose_stamp = row['timestamp_ms']
            # Every layer sees the pose to timestamp-pair its own range samples.
            # Only matched, settled samples may create occupancy in that layer.
            transition = (row.get('mapping_paused') or row.get('phase') == 'height_change' or
                          (self.navigation or {}).get('state') == 'CHANGE_HEIGHT')
            for height, m in self.maps.items():
                sample = dict(row, hovering=bool(row.get('hovering') and
                    abs(height-z) <= self.tolerance and not transition))
                m.update(sample, now)
            return
        if name == 'cf2_ranges':
            self.ranges, self.last_ranges_at = dict(data), now
            z = self.pose['stateEstimate.z'] if self.pose else math.nan
            transition = (row.get('mapping_paused') or row.get('phase') == 'height_change' or
                          (self.navigation or {}).get('state') == 'CHANGE_HEIGHT')
            for height, m in self.maps.items():
                m.update(dict(row, hovering=bool(row.get('hovering') and
                    abs(height-z) <= self.tolerance and not transition)), now)

    def inside(self, x, y, padding=0):
        if not self.area or not self.start:
            return True
        return (abs(x-self.start[0])+padding <= self.area[0]/2 and
                abs(y-self.start[1])+padding <= self.area[1]/2)

    @synchronized
    def planning_snapshot(self, now=None):
        return self.current.snapshot(now)

    @synchronized
    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        current = self.current.snapshot(now)
        key = (self.grid_origin, self.start, self.area)
        if self.grid_origin and key != self.mask_key:
            ox, oy = self.grid_origin
            self.mask = [self.inside(ox+(i % self.width+.5)*self.resolution,
                                oy+(i // self.width+.5)*self.resolution, self.resolution/2)
                    for i in range(self.width*self.width)]
            self.mask_key = key
        mask = self.mask
        layers = []
        recent = []
        for index, height in enumerate(self.heights):
            s = current if index == self.active else self.maps[height].snapshot(now)
            recent.extend(p for p in s['recent_hits'] if self.inside(*p))
            layers.append({'index': index, 'height_m': height, 'tolerance_m': self.tolerance,
                'data': [v if not mask or mask[i] else -1 for i, v in enumerate(s['data'])],
                'presentation_data': [v if not mask or mask[i] else -1
                                      for i, v in enumerate(s['presentation_data'])],
                'trail': s['trail'], 'stats': s['stats']})
        composite = []
        for values in zip(*(layer['presentation_data'] for layer in layers)):
            composite.append(100 if 100 in values else (0 if 0 in values else -1))
        current.update(origin=self.grid_origin, start=self.start, pose=self.pose,
            navigation=self.navigation, ranges_mm=self.ranges, trail=list(self.trail),
            layers=layers, active_layer=self.active, survey_heights=list(self.heights),
            area=self.area, presentation_data=composite,
            data=layers[self.active]['data'],
            recent_hits=recent,
            age_seconds=None if self.last_pose_at is None or self.last_ranges_at is None else
                max(now-self.last_pose_at, now-self.last_ranges_at),
            composite_is_visualization_only=True, shared_frame_aligned=False,
            map_validated_for_drone1=False, height_reference='Session EKF Z in metres')
        return current
