"""Pruebas de range_filter.py (TP-20). Ejecutar: python3 -m pytest test/test_range_filter.py"""
import math

from range_processor.range_filter import (
    SENSORS, RangeFilter, RangeFilterConfig, classify, time_to_collision,
)

CONFIG = RangeFilterConfig(range_min=0.02, range_max=4.0, filter_window=3, max_valid_jump=1.0)


def _raw(front):
    raw = {name: 2.0 for name in SENSORS}
    raw['front'] = front
    return raw


def test_classify():
    assert classify(math.nan, CONFIG) == 'invalid'
    assert classify(math.inf, CONFIG) == 'no_return'
    assert classify(5.0, CONFIG) == 'no_return'
    assert classify(-math.inf, CONFIG) == 'too_close'
    assert classify(0.01, CONFIG) == 'too_close'
    assert classify(1.0, CONFIG) == 'valid'


def test_short_spike_is_rejected():
    f = RangeFilter(CONFIG)
    for i in range(100):
        t = i * 0.05
        out = f.update(t, _raw(0.1 if 2.0 <= t <= 2.2 else 2.0))
        assert out['front'] == 2.0
    assert f.rejected_spikes == 1


def test_real_obstacle_is_accepted_after_confirm_time():
    f = RangeFilter(CONFIG)
    accepted_at = None
    for i in range(60):
        t = i * 0.05
        out = f.update(t, _raw(0.5 if t >= 1.0 else 2.0))
        if accepted_at is None and out['front'] == 0.5:
            accepted_at = t
    assert accepted_at is not None
    assert 1.25 <= accepted_at <= 1.4


def test_closing_rate_and_ttc():
    f = RangeFilter(CONFIG)
    for i in range(20):
        t = i * 0.05
        out = f.update(t, _raw(2.0 - 0.5 * t))
    rate = f.closing_rate('front')
    assert abs(rate + 0.5) < 0.05
    assert abs(time_to_collision(out['front'], rate) - out['front'] / 0.5) < 0.2


def test_too_close_and_nan():
    f = RangeFilter(CONFIG)
    for i in range(20):
        t = i * 0.05
        raw = _raw(-math.inf if t >= 0.2 else 0.1)
        raw['left'] = math.nan if t >= 0.2 else 1.0
        out = f.update(t, raw)
    assert out['front'] == -math.inf
    assert math.isnan(out['left'])


def test_ttc_cases():
    assert time_to_collision(1.0, None) == math.inf
    assert time_to_collision(1.0, 0.2) == math.inf
    assert time_to_collision(math.inf, -1.0) == math.inf
    assert time_to_collision(-math.inf, -1.0) == 0.0
    assert time_to_collision(1.0, -0.5) == 2.0


def test_flicker_at_max_range_is_not_a_spike():
    """Crazyswarm2 corta en 3.49 m: 3.48 <-> +inf es salir de alcance, no un spike."""
    f = RangeFilter(CONFIG)
    for i in range(40):
        f.update(i * 0.05, _raw(3.48 if i % 2 else math.inf))
    assert f.rejected_spikes == 0
