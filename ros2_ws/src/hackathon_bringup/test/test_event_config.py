"""Event consistency regressions; they neither open a radio nor start ROS."""
from pathlib import Path
import pytest
import yaml
from hackathon_bringup.event_config import validate_event


@pytest.fixture
def event():
    config = Path(__file__).resolve().parents[1]/'config'
    return [yaml.safe_load(path.read_text()) for path in
            (config/'event.yaml', config/'event_hardware.yaml', config/'missions/event.yaml')]


def test_draft_is_coherent_and_has_explicit_pending_identity(event):
    errors, pending = validate_event(*event)
    assert not errors and len(pending) == 4


def test_filled_hardware_matches_adapter(event):
    params, hardware, mission = event
    robot = hardware['robots'].pop('ROBOT_ID_TODO')
    robot.update(enabled=True, uri='radio://0/80/2M/E7E7E7E701',type='delivered_type')
    hardware['robots']['cf01'] = robot
    hardware['robot_types']['delivered_type'] = hardware['robot_types'].pop('EVENT_TYPE_TODO')
    params['cf_adapter']['ros__parameters']['robot_name'] = 'cf01'
    assert validate_event(*event) == ([], [])


@pytest.mark.parametrize('section,key,value', [
    ('/**','z_nominal',2.0),('/**','warning_distance',.1),
    ('local_navigator','nominal_speed',.9),('mission_manager','autostart',True),
    ('safety_supervisor','geofence',[0,0,1,0,2,2]),
    ('local_world_model','map_width',1.0),('range_processor','range_max',.01),
    ('/**','state_timeout',float('nan'))])
def test_invalid_envelope_limits_or_mode_rejected(event,section,key,value):
    event[0][section]['ros__parameters'][key] = value
    assert validate_event(*event)[0]


def test_mission_frame_and_waypoint_must_match_envelope(event):
    event[2].update(frame_id='other',waypoints=[[10,0,.5]])
    errors, _ = validate_event(*event)
    assert any('Mission frame' in error for error in errors)
    assert any('Waypoint outside the geofence' in error for error in errors)


def test_hardware_identity_and_frame_must_match(event):
    event[0]['cf_adapter']['ros__parameters']['robot_name'] = 'different'
    assert any('robot_name' in error for error in validate_event(*event)[0])


def test_ready_cli_rejects_unfilled_template():
    import subprocess
    root = Path(__file__).resolve().parents[4]
    result = subprocess.run(['/usr/bin/python3',str(root/'scripts/verify_event_config.py'),'--ready'],
                            capture_output=True,text=True)
    assert result.returncode == 1
    assert 'PENDING:' in result.stdout
