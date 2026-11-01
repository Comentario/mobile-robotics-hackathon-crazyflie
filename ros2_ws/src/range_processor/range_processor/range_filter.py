"""Range validation and filtering (TP-19, TP-20) and closing rate d_dot (TP-36).

REP-117 convention in the whole API: +inf no return, -inf too close, NaN invalid.

Pure logic (rule 6 of interfaces.md): no rclpy, tested with pytest.

What update() does with every sensor, in every message
------------------------------------------------------
  1. Classify the reading (classify):
       NaN                    -> 'invalid'    (not used)
       +inf or > range_max    -> 'no_return'  (nothing ahead, treated as +inf)
       -inf or < range_min    -> 'too_close'  (something touching, treated as 0 m)
       otherwise              -> 'valid'
  2. Spike rejection: if the reading jumps more than max_valid_jump from the accepted
     value, it is NOT believed right away. It is stored as a "candidate" and only
     accepted if the following readings confirm it for spike_confirm_time seconds.
     A spike of 1-4 samples (T08) is discarded; a real obstacle is accepted with a
     delay of spike_confirm_time.
  3. Median of the last filter_window accepted readings (removes noise without
     adding much delay: with 3 samples at 20 Hz it is ~0.05 s).
  4. If the sensor gives NaN for more than invalid_hold consecutive seconds, the
     output is NaN; before that, the last filtered value is held.

closing_rate (d_dot): slope (linear regression) of the filtered distance over the
last rate_window s. Negative = the obstacle is approaching.
"""
from collections import deque
from dataclasses import dataclass
import math

SENSORS = ('front', 'back', 'left', 'right', 'up', 'down')


@dataclass
class RangeFilterConfig:
    range_min: float
    range_max: float
    filter_window: int      # samples for the median; small so it adds little delay
    max_valid_jump: float   # m between samples; larger is treated as a spike
    spike_confirm_time: float = 0.3   # s a jump must persist to be accepted
    invalid_hold: float = 0.3         # s the last value is held when NaN arrives
    rate_window: float = 0.3          # s of history to estimate d_dot
    rate_min_samples: int = 3         # minimum samples to provide a d_dot


def classify(value, config):
    """Return 'valid', 'no_return', 'too_close' or 'invalid'."""
    if math.isnan(value):
        return 'invalid'
    if value == math.inf or value > config.range_max:
        return 'no_return'
    if value == -math.inf or value < config.range_min:
        return 'too_close'
    return 'valid'


def _to_distance(value, kind):
    """Reading -> internal distance: too_close = 0 m, no_return = +inf."""
    if kind == 'too_close':
        return 0.0
    if kind == 'no_return':
        return math.inf
    return float(value)


class _SensorState:
    def __init__(self, window):
        self.accepted = deque(maxlen=window)   # accepted distances (for the median)
        self.reference = None                  # last accepted value (to detect jumps)
        self.candidate = None                  # (t_start, [values]) of an unconfirmed jump
        self.output = math.nan                 # last published filtered value
        self.last_ok_t = None                  # time of the last non-NaN reading
        self.history = deque()                 # (t, filtered distance) for d_dot


class RangeFilter:

    def __init__(self, config):
        self.config = config
        self._sensors = {name: _SensorState(max(1, config.filter_window)) for name in SENSORS}
        self.rejected_spikes = 0               # counter, useful for logs and tests

    def update(self, t, raw):
        """raw: dict sensor -> m. Returns a dict sensor -> filtered m (same format)."""
        out = {}
        for name in SENSORS:
            s = self._sensors[name]
            value = raw.get(name, math.nan)
            kind = classify(value, self.config)

            if kind == 'invalid':
                # Hold the last value for a while; then NaN.
                if s.last_ok_t is None or t - s.last_ok_t > self.config.invalid_hold:
                    s.output = math.nan
                    s.history.clear()
                out[name] = s.output
                continue

            s.last_ok_t = t
            distance = _to_distance(value, kind)
            if kind == 'too_close':
                s.accepted.clear()
                s.accepted.append(0.0)
                s.reference, s.candidate = 0.0, None
                s.output = -math.inf
                s.history.clear()
                out[name] = s.output
                continue
            self._accept_or_hold(t, s, distance)

            median = _median(s.accepted)
            s.output = -math.inf if median < self.config.range_min else median
            self._record_history(t, s, median)
            out[name] = s.output
        return out

    def closing_rate(self, sensor):
        """Estimated sensor d_dot in m/s, or None if it is not stable."""
        history = self._sensors[sensor].history
        if len(history) < self.config.rate_min_samples:
            return None
        ts = [h[0] for h in history]
        ds = [h[1] for h in history]
        t_mean = sum(ts) / len(ts)
        d_mean = sum(ds) / len(ds)
        den = sum((ti - t_mean) ** 2 for ti in ts)
        if den <= 0.0:
            return None
        return sum((ti - t_mean) * (di - d_mean) for ti, di in zip(ts, ds)) / den

    # ------------------------------------------------------------------
    def _accept_or_hold(self, t, s, distance):
        """Step 2 of the docstring: accept the reading or treat it as a possible spike."""
        jump = self.config.max_valid_jump
        # For comparison, "no return" (+inf) counts as range_max: going from 3.48 m to
        # +inf is the obstacle leaving the range, not a spike (the Crazyswarm2
        # LaserScan cuts at 3.49 m and with noise it flickers between 3.48 and +inf).
        cmp_now = min(distance, self.config.range_max)
        cmp_ref = None if s.reference is None else min(s.reference, self.config.range_max)
        if cmp_ref is None or abs(cmp_now - cmp_ref) <= jump:
            # Reading consistent with the accepted value: accept it and forget any candidate.
            if s.candidate is not None:
                self.rejected_spikes += 1
                s.candidate = None
            s.accepted.append(distance)
            s.reference = distance
            return

        # Large jump. Does it continue a consistent candidate?
        last = min(s.candidate[1][-1], self.config.range_max) if s.candidate else None
        if s.candidate is not None and abs(cmp_now - last) <= jump:
            s.candidate[1].append(distance)
        else:
            if s.candidate is not None:
                self.rejected_spikes += 1
            s.candidate = (t, [distance])

        if t - s.candidate[0] >= self.config.spike_confirm_time:
            # Confirmed: it is a real change (a new obstacle or one that disappears).
            s.accepted.clear()
            s.accepted.extend(s.candidate[1][-s.accepted.maxlen:])
            s.reference = distance
            s.candidate = None
            s.history.clear()      # the previous d_dot is no longer valid

    def _record_history(self, t, s, distance):
        """Store the filtered distance for d_dot (finite values only)."""
        if not math.isfinite(distance):
            s.history.clear()
            return
        s.history.append((t, distance))
        while s.history and t - s.history[0][0] > self.config.rate_window:
            s.history.popleft()


def _median(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return math.nan
    mid = n // 2
    if n % 2:
        return ordered[mid]
    low, high = ordered[mid - 1], ordered[mid]
    if math.isinf(low) or math.isinf(high):
        return high if math.isinf(high) else low
    return (low + high) / 2.0


def time_to_collision(distance, closing_rate):
    """TTC = distance / -d_dot if the obstacle is approaching; math.inf otherwise."""
    if closing_rate is None or not math.isfinite(closing_rate) or closing_rate >= 0.0:
        return math.inf
    if math.isnan(distance) or distance == math.inf:
        return math.inf
    if distance <= 0.0:          # -inf (too close) or 0
        return 0.0
    return distance / -closing_rate
