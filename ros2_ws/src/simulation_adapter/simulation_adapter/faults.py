"""Controlled fault injection used by simulation scenarios."""

import copy
import math


FAULT_TYPES = (
    'range_spike',
    'range_dropout',
    'pose_delay',
    'pose_freeze',
    'pose_jump',
    'pose_dropout',
    'command_pause',
)


class FaultInjector:
    """Apply deterministic faults defined in scenario YAML files."""

    def __init__(self, faults):
        self.faults = faults or []

        for fault in self.faults:
            fault_type = fault.get('type')

            if fault_type not in FAULT_TYPES:
                raise ValueError(
                    f'Unsupported fault type: {fault_type}'
                )

    @staticmethod
    def _is_active(t, fault):
        start = float(
            fault.get('t_start', 0.0)
        )

        end = float(
            fault.get('t_end', math.inf)
        )

        return start <= t <= end

    def active(self, t, fault_type):
        """Return active faults of the requested type."""
        return [
            fault
            for fault in self.faults
            if (
                fault.get('type') == fault_type
                and self._is_active(t, fault)
            )
        ]

    def apply_to_ranges(self, t, ranges):
        """Apply range spikes and dropouts."""
        result = dict(ranges)

        for fault in self.active(t, 'range_spike'):
            sensor = fault.get('sensor', 'all')
            value = float(fault['value'])

            if sensor == 'all':
                for name in result:
                    result[name] = value

            elif sensor in result:
                result[sensor] = value

        for fault in self.active(t, 'range_dropout'):
            sensor = fault.get('sensor', 'all')

            if sensor == 'all':
                for name in result:
                    result[name] = math.nan

            elif sensor in result:
                result[sensor] = math.nan

        return result

    @staticmethod
    def _history_at_or_before(pose_history, target_time):
        candidates = [
            (stamp, pose)
            for stamp, pose in pose_history
            if stamp <= target_time
        ]

        if not candidates:
            return None

        _, pose = max(
            candidates,
            key=lambda entry: entry[0],
        )

        return copy.deepcopy(pose)

    @staticmethod
    def _time_at_or_before(pose_history, target_time):
        return max(stamp for stamp, _ in pose_history if stamp <= target_time)

    def apply_to_pose(self, t, pose, pose_history):
        """Apply delay, freeze, jump, and dropout faults.

        A delayed or frozen pose carries `stamp_t`, the simulation time of the sample.
        """
        if self.active(t, 'pose_dropout'):
            return None

        result = copy.deepcopy(pose)

        delays = self.active(t, 'pose_delay')

        if delays:
            delay = float(delays[0].get('delay', 0.0))

            delayed = self._history_at_or_before(
                pose_history,
                t - delay,
            )

            if delayed is None:
                return None

            result = delayed
            result['stamp_t'] = self._time_at_or_before(pose_history, t - delay)

        freezes = self.active(t, 'pose_freeze')

        if freezes:
            freeze_start = float(
                freezes[0].get('t_start', 0.0)
            )

            frozen = self._history_at_or_before(
                pose_history,
                freeze_start,
            )

            if frozen is not None:
                result = frozen
                # A frozen estimator stops producing new measurements: keep the
                # sample time so consumers see the pose age grow (state_monitor).
                # A delayed sample keeps its measurement time in the same way.
                result['stamp_t'] = self._time_at_or_before(pose_history, freeze_start)

        for fault in self.active(t, 'pose_jump'):
            offset = fault.get(
                'offset',
                [0.0, 0.0, 0.0],
            )

            result['x'] += float(offset[0])
            result['y'] += float(offset[1])
            result['z'] += float(offset[2])

        return result

    def drop_command(self, t):
        """Return True while command-pause injection is active."""
        return bool(
            self.active(t, 'command_pause')
        )