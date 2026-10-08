"""Pruebas de ranges.py (TP-09). Ejecutar: python3 -m pytest test/test_ranges.py"""
import math

from cf_adapter.ranges import extra_to_ranges, scan_to_ranges

# Formato real de Crazyswarm2 1.0.7: [back, right, front, left], angle_min = -pi, paso pi/2.
ANGLE_MIN = -math.pi
STEP = math.pi / 2.0


def test_scan_order_of_crazyswarm2():
    out = scan_to_ranges([0.5, 2.0, 1.45, 3.0], ANGLE_MIN, STEP, 0.01, 3.49)
    assert out == {'back': 0.5, 'right': 2.0, 'front': 1.45, 'left': 3.0}


def test_scan_inf_nan_and_too_close():
    out = scan_to_ranges([math.inf, math.nan, 0.0, 5.0], ANGLE_MIN, STEP, 0.01, 3.49)
    assert out['back'] == math.inf
    assert math.isnan(out['right'])
    assert out['front'] == -math.inf
    assert out['left'] == math.inf


def test_extra_mm_to_m():
    out = extra_to_ranges([1500.0, 8000.0], 0.01, 3.49)
    assert out == {'up': 1.5, 'down': math.inf}
    assert all(math.isnan(v) for v in extra_to_ranges([], 0.01, 3.49).values())
