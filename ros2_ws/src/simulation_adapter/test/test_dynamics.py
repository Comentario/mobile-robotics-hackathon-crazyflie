"""Unit tests for the simulator kinematic model."""

import math
from types import SimpleNamespace

import pytest

from simulation_adapter.dynamics import KinematicDrone


def make_setpoint(
    mode=0,
    x=0.0,
    y=0.0,
    z=0.5,
    vx=0.0,
    vy=0.0,
    vz=0.0,
    yaw=0.0,
):
    return SimpleNamespace(
        mode=mode,
        position=SimpleNamespace(
            x=x,
            y=y,
            z=z,
        ),
        velocity=SimpleNamespace(
            x=vx,
            y=vy,
            z=vz,
        ),
        yaw=yaw,
    )


def simulate(drone, start_time, duration, dt=0.01):
    steps = int(duration / dt)

    for index in range(steps):
        drone.step(
            start_time + index * dt,
            dt,
        )


def test_reaches_position_setpoint():
    drone = KinematicDrone(
        max_speed=1.0,
        max_accel=2.0,
        command_watchdog=10.0,
    )

    drone.takeoff(0.5, 1.0)
    simulate(drone, 0.0, 2.0)

    command = make_setpoint(
        mode=0,
        x=1.0,
        y=0.5,
        z=0.5,
    )

    drone.set_command(2.0, command)

    simulate(drone, 2.0, 4.0)

    assert drone.state.x == pytest.approx(
        1.0,
        abs=0.03,
    )

    assert drone.state.y == pytest.approx(
        0.5,
        abs=0.03,
    )

    assert drone.state.z == pytest.approx(
        0.5,
        abs=0.03,
    )


def test_respects_max_speed():
    max_speed = 0.4

    drone = KinematicDrone(
        max_speed=max_speed,
        max_accel=10.0,
        command_watchdog=10.0,
    )

    drone.takeoff(0.5, 1.0)
    simulate(drone, 0.0, 2.0)

    command = make_setpoint(
        mode=0,
        x=10.0,
        y=10.0,
        z=1.0,
    )

    drone.set_command(2.0, command)

    for index in range(200):
        drone.step(
            2.0 + index * 0.01,
            0.01,
        )

        speed = math.sqrt(
            drone.state.vx ** 2
            + drone.state.vy ** 2
            + drone.state.vz ** 2
        )

        assert speed <= max_speed + 1e-9


def test_hover_after_command_watchdog():
    drone = KinematicDrone(
        max_speed=1.0,
        max_accel=2.0,
        command_watchdog=0.5,
    )

    drone.takeoff(0.5, 1.0)
    simulate(drone, 0.0, 2.0)

    command = make_setpoint(
        mode=1,
        z=0.5,
        vx=0.5,
    )

    drone.set_command(2.0, command)

    simulate(drone, 2.0, 3.0)

    speed = math.sqrt(
        drone.state.vx ** 2
        + drone.state.vy ** 2
        + drone.state.vz ** 2
    )

    assert speed < 0.05
    assert drone.state.flying


def test_emergency_stop_falls():
    drone = KinematicDrone(
        max_speed=1.0,
        max_accel=2.0,
        command_watchdog=0.5,
    )

    drone.takeoff(0.5, 1.0)
    simulate(drone, 0.0, 2.0)

    assert drone.state.z > 0.4

    drone.emergency_stop()

    simulate(drone, 2.0, 2.0)

    assert drone.state.z == pytest.approx(0.0)
    assert not drone.state.flying