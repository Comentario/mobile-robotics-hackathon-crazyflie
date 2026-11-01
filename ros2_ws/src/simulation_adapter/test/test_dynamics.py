"""TP-10: flight cycle and commands of the pure model, without ROS."""
import math
import pytest
from simulation_adapter.dynamics import DroneState, KinematicDrone, MotionCommand


def drone(state=None):
    return KinematicDrone(1.0, 2.0, 0.5, state, response_time=0.2)


def advance(model, seconds, start=0.0, dt=0.01):
    for tick in range(round(seconds/dt)):
        model.step(start + (tick+1)*dt, dt)
    return model.state


def test_reaches_position_setpoint():
    model = drone(DroneState(z=1))
    for tick in range(300):
        model.set_command(tick*0.01, MotionCommand(0, (1, 0, 1)))
        model.step((tick+1)*0.01, 0.01)
    assert model.state.x == pytest.approx(1, abs=0.01)
    assert abs(model.state.vx) < 0.02


def test_respects_max_speed():
    model = drone(DroneState(z=1))
    for tick in range(300):
        previous = model.state
        model.set_command(tick*0.01, MotionCommand(0, (3, 4, 2)))
        model.step((tick+1)*0.01, 0.01)
        speed = math.hypot(model.state.vx, model.state.vy, model.state.vz)
        change = (model.state.vx-previous.vx, model.state.vy-previous.vy, model.state.vz-previous.vz)
        assert speed <= model.max_speed + 1e-12
        assert math.hypot(*change) <= model.max_accel*0.01 + 1e-12


def test_hover_after_command_watchdog():
    model = drone(DroneState(z=1))
    model.set_command(0, MotionCommand(0, (2, 0, 1)))
    advance(model, 0.6)
    hover = model.target
    assert model.mode == 2 and model.last_command_time is None
    advance(model, 4, start=0.6)
    assert model.target == hover
    assert model.state.x == pytest.approx(hover[0], abs=0.01)
    assert abs(model.state.vx) < 0.01


def test_takeoff_then_land_without_command_stream():
    model = drone()
    model.takeoff(0.5, 1.0)
    assert model.phase == 'TAKING_OFF'
    advance(model, 3)
    assert model.phase == 'FLYING' and model.state.flying
    assert model.state.z == pytest.approx(0.5, abs=0.01)
    model.land(1.0)
    assert model.phase == 'LANDING'
    advance(model, 3, start=3)
    assert model.phase == 'GROUND' and not model.state.flying
    assert model.state.z == 0 and model.state.vz == 0
    model.takeoff(0.3, 1.0)
    advance(model, 3, start=6)
    assert model.phase == 'FLYING'


def test_ground_and_maneuvers_block_navigation_commands():
    model = drone()
    command = MotionCommand(0, (2, 0, 1))
    with pytest.raises(ValueError):
        model.set_command(0, command)
    model.takeoff(0.5, 1)
    with pytest.raises(ValueError):
        model.set_command(0, command)
    advance(model, 3)
    model.land(1)
    with pytest.raises(ValueError):
        model.set_command(3, command)


def test_landing_can_interrupt_takeoff_and_repeated_land_does_not_reset_it():
    model = drone()
    model.takeoff(0.5, 2)
    advance(model, 0.5)
    model.land(1)
    advance(model, 0.1, start=0.5)
    elapsed = model.maneuver_elapsed
    model.land(5)
    assert model.maneuver_elapsed == elapsed
    assert model.maneuver_duration == 1
    advance(model, 3, start=0.6)
    assert model.phase == 'GROUND'


def test_emergency_stop_falls():
    model = KinematicDrone(1, 2, 0.5, DroneState(z=1), response_time=1)
    model.emergency_stop()
    model.step(0.1, 0.1)
    assert model.state.z == pytest.approx(1 - 0.5*9.81*0.1**2)
    assert model.state.vz == pytest.approx(-0.981)
    advance(model, 1, start=0.1)
    assert model.state.z == 0 and not model.state.flying
    assert model.phase == 'EMERGENCY'
    for action in (lambda: model.takeoff(0.5, 1), lambda: model.land(1),
                   lambda: model.set_command(1.1, MotionCommand(0, (2, 0, 1)))):
        with pytest.raises(ValueError):
            action()
    advance(model, 1, start=1.1)
    assert model.state.z == 0 and model.state.vz == 0


@pytest.mark.parametrize('height,duration', [(0, 1), (-1, 1), (math.nan, 1), (0.5, 0), (0.5, math.inf)])
def test_invalid_takeoff_does_not_change_phase(height, duration):
    model = drone()
    with pytest.raises(ValueError):
        model.takeoff(height, duration)
    assert model.phase == 'GROUND'


def test_takeoff_in_flight_and_invalid_landing_are_rejected():
    model = drone(DroneState(z=1))
    with pytest.raises(ValueError):
        model.takeoff(0.5, 1)
    with pytest.raises(ValueError):
        model.land(math.nan)
    assert model.phase == 'FLYING'


def test_negative_altitude_and_invalid_configuration_are_rejected():
    with pytest.raises(ValueError):
        drone(DroneState(z=-1))
    with pytest.raises(ValueError):
        KinematicDrone(1, 0, 0.5)
    model = drone(DroneState(z=1))
    with pytest.raises(ValueError):
        model.set_command(0, MotionCommand(0, (0, 0, -1)))


def test_velocity_xy_altitude_acceleration_and_watchdog():
    model = drone(DroneState(z=1))
    for tick in range(200):
        previous = model.state
        model.set_command(tick*.01, MotionCommand(1, (0,0,1), velocity=(0.4,0.3,0)))
        model.step((tick+1)*.01,.01)
        assert math.hypot(model.state.vx-previous.vx, model.state.vy-previous.vy, model.state.vz-previous.vz) <= .02+1e-12
    assert model.state.x > 0.5 and model.state.y > 0.4
    assert model.state.z == 1
    assert model.state.vx == pytest.approx(0.4)
    advance(model, 3, start=2)
    assert model.mode == 2 and abs(model.state.vx) < .01


def test_yaw_shortest_path_rate_and_hover_latch():
    model = drone(DroneState(z=1,yaw=math.pi-.1))
    model.set_command(0,MotionCommand(0,(0,0,1),yaw=-math.pi+.1))
    model.step(.01,.01)
    assert model.state.yaw_rate == pytest.approx(model.max_yaw_rate)
    for tick in range(30):
        model.set_command(.01*(tick+1),MotionCommand(0,(0,0,1),yaw=-math.pi+.1))
        model.step(.01*(tick+2),.01)
    assert model.state.yaw == pytest.approx(-math.pi+.1)
    model.set_command(.4,MotionCommand(2))
    target = model.target_yaw
    advance(model,1,start=.4)
    assert model.target_yaw == target


def test_velocity_vertical_rejected_and_large_velocity_limited():
    model = drone(DroneState(z=1))
    with pytest.raises(ValueError):
        model.set_command(0,MotionCommand(1,(0,0,1),velocity=(0,0,1)))
    model.set_command(0,MotionCommand(1,(0,0,1),velocity=(100,100,0)))
    assert math.hypot(*model.velocity_reference) == pytest.approx(1)
