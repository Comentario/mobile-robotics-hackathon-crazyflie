"""Controlled fault injection for the TP-41 campaign and the T08–T16 test matrix.

Format in config/scenarios/*.yaml:
  faults:
    - {type: range_spike,    sensor: front, value: 0.1, t_start: 5.0, t_end: 5.2}
    - {type: range_dropout,  sensor: all,               t_start: 5.0, t_end: 8.0}
    - {type: pose_delay,     delay: 0.5,                t_start: 5.0, t_end: 10.0}
    - {type: pose_freeze,                               t_start: 5.0, t_end: 10.0}
    - {type: pose_jump,      offset: [0.5, 0.0, 0.0],   t_start: 5.0, t_end: 5.0}
    - {type: pose_dropout,                              t_start: 5.0}   # Lighthouse lost
    - {type: command_pause,                             t_start: 5.0, t_end: 8.0}

Pure logic (rule 6 of interfaces.md): no rclpy. Times t are simulation seconds
since the simulator started.

How every pose fault is interpreted (what the stack would see with a real drone)
--------------------------------------------------------------------------------
  pose_dropout  No pose is published. Mimics losing the radio or the Lighthouse.
  pose_delay    The pose from 'delay' s ago is published, with its original (old) time.
                Mimics radio latency or an overloaded PC.
  pose_freeze   The same pose (identical values) is always published with a new time.
                Mimics a hung estimator: data arrives, but it does not change.
  pose_jump     'offset' is added to the pose. If t_end == t_start (or there is no
                t_end), the offset stays forever from t_start: mimics an EKF reset or
                the Lighthouse suddenly "relocating" the drone.

Faults are applied ONLY to the published pose (/hk/adapter/odom), never to the real
pose (/hk/sim/ground_truth), as docs/simulations.md requires.

Format of one pose sample (dict): {'t', 'stamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz'}
  t      simulation time (s)
  stamp  ROS time of the measurement (s), the one that goes into header.stamp
"""
import math

FAULT_TYPES = (
    'range_spike', 'range_dropout', 'pose_delay', 'pose_freeze',
    'pose_jump', 'pose_dropout', 'command_pause',
)

RANGE_SENSORS = ('front', 'back', 'left', 'right', 'up', 'down')


class FaultInjector:

    def __init__(self, faults):
        """faults: list of dicts as in the module docstring."""
        self.faults = list(faults or [])
        for fault in self.faults:
            if fault.get('type') not in FAULT_TYPES:
                raise ValueError(f'Unknown fault type: {fault.get("type")}')
        # Frozen pose while a pose_freeze lasts (stored when it starts).
        self._frozen = None

    # ------------------------------------------------------------------
    def active(self, t, fault_type):
        """Faults of that type active at t."""
        result = []
        for fault in self.faults:
            if fault['type'] != fault_type:
                continue
            t_start = fault.get('t_start', 0.0)
            t_end = fault.get('t_end', math.inf)
            if fault_type == 'pose_jump' and t_end <= t_start:
                t_end = math.inf      # permanent jump (see docstring)
            if t_start <= t <= t_end:
                result.append(fault)
        return result

    def active_types(self, t):
        """Names of all fault types active at t (for the node log)."""
        return sorted({f['type'] for f in self.faults if self.active(t, f['type'])})

    # ------------------------------------------------------------------
    def apply_to_ranges(self, t, ranges):
        """Return the ranges (dict sensor -> m) modified by spikes/dropouts.

        Returns None if a range_dropout with sensor 'all' is active (nothing is
        published). A single-sensor dropout marks it as NaN (REP-117: invalid).
        """
        out = dict(ranges)
        for fault in self.active(t, 'range_dropout'):
            sensor = fault.get('sensor', 'all')
            if sensor == 'all':
                return None
            out[sensor] = math.nan
        for fault in self.active(t, 'range_spike'):
            sensors = RANGE_SENSORS if fault.get('sensor') == 'all' else (fault['sensor'],)
            for sensor in sensors:
                out[sensor] = float(fault['value'])
        return out

    def apply_to_pose(self, t, pose, pose_history):
        """Return the pose to publish (delayed, frozen, jumped) or None if nothing is published.

        pose: current sample (dict). pose_history: previous samples, from the oldest
        to the newest (must include the current one).
        """
        # 1. Dropout: nothing is published.
        if self.active(t, 'pose_dropout'):
            return None

        out = dict(pose)

        # 2. Delay: an old sample is published, with its old time.
        delays = self.active(t, 'pose_delay')
        if delays:
            delay = max(float(f.get('delay', 0.5)) for f in delays)
            old = self._sample_at(pose_history, t - delay)
            if old is None:
                return None          # not enough history yet
            out = dict(old)

        # 3. Freeze: always the same values, but with the current time.
        if self.active(t, 'pose_freeze'):
            if self._frozen is None:
                self._frozen = dict(out)
            frozen = dict(self._frozen)
            frozen['t'] = out['t']
            frozen['stamp'] = out['stamp']
            out = frozen
        else:
            self._frozen = None

        # 4. Jump: offset added to the position.
        for fault in self.active(t, 'pose_jump'):
            dx, dy, dz = (list(fault.get('offset', [0.0, 0.0, 0.0])) + [0.0, 0.0, 0.0])[:3]
            out['x'] += dx
            out['y'] += dy
            out['z'] += dz

        return out

    def drop_command(self, t):
        """True if the command received at t must be ignored (command_pause)."""
        return bool(self.active(t, 'command_pause'))

    # ------------------------------------------------------------------
    @staticmethod
    def _sample_at(history, t_target):
        """Last sample with t <= t_target, or None if there is none."""
        candidate = None
        for sample in history:
            if sample['t'] <= t_target:
                candidate = sample
            else:
                break
        return candidate
