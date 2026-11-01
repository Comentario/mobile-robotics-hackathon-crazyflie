import math
from safety_supervisor.supervisor import (SafetySupervisor,SupervisorConfig,SafetyInputs,HOVER,NORMAL,effective_clearance)


def inputs(**changes):
    values=dict(now=0,localization=0,state_stale=False,ranges_stale=False,command_stale=False,
                min_horizontal_range=math.inf,min_ttc=math.inf,z=.5,planner_no_path=False,outside_geofence=False)
    values.update(changes)
    return SafetyInputs(**values)


def test_invalid_required_ranges_hover():
    supervisor=SafetySupervisor(SupervisorConfig())
    assert supervisor.evaluate(inputs(ranges_stale=True))[0]==HOVER


def test_too_close_is_not_ignored():
    supervisor=SafetySupervisor(SupervisorConfig())
    clearance=effective_clearance(-math.inf,None,supervisor.config)
    assert supervisor.evaluate(inputs(min_horizontal_range=clearance))[0]==HOVER


def test_valid_no_return_is_clear():
    assert SafetySupervisor(SupervisorConfig()).evaluate(inputs())[0]==NORMAL
