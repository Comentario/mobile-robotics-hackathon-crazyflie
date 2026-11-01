"""Measurement health: motion is not the same as freshness."""
import math
from state_monitor.health import HealthConfig,LocalizationHealthMonitor,GOOD,LOST,DEGRADED


def monitor(): return LocalizationHealthMonitor(HealthConfig(.3,.15,.3,2))


def test_stationary_new_timestamps_are_good():
    m=monitor()
    for i in range(51): m.update(i*.02,(0,0,.5))
    assert m.evaluate(1)[0]==GOOD


def test_repeated_timestamp_and_dropout_are_lost():
    m=monitor();m.update(0,(0,0,.5));m.update(0,(0,0,.5))
    assert m.evaluate(.2)[0]==DEGRADED
    assert m.evaluate(.4)[0]==LOST


def test_nan_pose_is_lost():
    m=monitor();m.update(0,(0,0,.5));m.update(.01,(math.nan,0,.5))
    assert m.evaluate(.01)[0]==LOST


def test_jump_is_degraded():
    m=monitor();m.update(0,(0,0,.5));m.update(.02,(1,0,.5))
    assert m.evaluate(.02)[0]==DEGRADED
