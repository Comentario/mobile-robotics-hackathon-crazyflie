"""Exercise /hk services and commands through cf_adapter into installed SIL.

Requires cffirmware bindings and ROS Humble; no hardware or radio is used.
"""
import tempfile
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cffirmware')
pytest.importorskip('crazyflie_sim')
import numpy as np
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from crazyflie_sim.crazyflie_server import CrazyflieServer
from hackathon_interfaces.msg import Setpoint
from hackathon_interfaces.srv import Land, Takeoff
from cf_adapter.cf_adapter_node import CrazyflieAdapter


def test_adapter_services_and_all_command_modes_in_sil():
    root = '/opt/ros/humble/share/crazyflie/config/'
    with open(root + 'crazyflies.yaml') as f:
        robots = yaml.safe_load(f)
    with open(root + 'server.yaml') as f:
        params = yaml.safe_load(f)['/crazyflie_server']['ros__parameters']
    params.update(robots)
    params['robot_description'] = '<robot name="$NAME"/>'
    for vis in params['sim']['visualizations'].values():
        vis['enabled'] = False
    params['sim']['visualizations']['rviz']['enabled'] = True
    assert [name for name, cfg in robots['robots'].items() if cfg['enabled']] == ['cf231']
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as config:
        yaml.safe_dump({
            '/crazyflie_server': {'ros__parameters': params},
            '/cf_adapter': {'ros__parameters': {'use_sim_time': True, 'state_source': 'tf'}},
        }, config)
        config.flush()
        rclpy.init(args=['--ros-args', '--params-file', config.name])
        server = CrazyflieServer()
        server.timer.cancel()
        adapter = CrazyflieAdapter()
    peer = Node('tp09_sil_driver', parameter_overrides=[Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    for node in (server, adapter, peer):
        executor.add_node(node)
    publisher = peer.create_publisher(Setpoint, '/hk/cmd/safe', 10)
    takeoff = peer.create_client(Takeoff, '/hk/takeoff')
    land = peer.create_client(Land, '/hk/land')
    visuals = server.visualizations
    ticks = 0

    def spin_until(predicate, timeout=3):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.001)
        assert predicate(), 'SIL transport did not finish'

    def advance(seconds, command=None):
        nonlocal ticks
        for _ in range(round(seconds/server.backend.dt)):
            server.visualizations = visuals if ticks % 40 == 0 else []
            server._timer_callback()
            ticks += 1
            if ticks % 40 == 0:
                # Drain clock/TF and command callbacks without running server's canceled timer.
                for _ in range(12):
                    executor.spin_once(timeout_sec=0)
            if command is not None and ticks % 100 == 0:
                command.header.stamp = adapter.get_clock().now().to_msg()
                publisher.publish(command)
        for _ in range(12):
            executor.spin_once(timeout_sec=0)
        return server.backend.uavs[0].state.pos.copy()

    def near(pos, target):
        assert np.linalg.norm(pos-np.array(target)) < 0.15, (pos, target)

    try:
        spin_until(lambda: takeoff.service_is_ready() and land.service_is_ready()
                   and adapter.cf_takeoff.service_is_ready() and adapter.cf_land.service_is_ready()
                   and adapter.cf_notify_stop.service_is_ready() and publisher.get_subscription_count())
        advance(0.1)
        spin_until(lambda: adapter.latest_state is not None)
        request = Takeoff.Request()
        request.height, request.duration = 0.5, 1.0
        future = takeoff.call_async(request)
        spin_until(future.done)
        assert future.result().success
        near(advance(2), [0, 0, 0.5])
        command = Setpoint()
        command.header.frame_id = 'map'
        command.mode = Setpoint.MODE_POSITION
        command.position.x, command.position.z = 0.3, 0.5
        near(advance(2, command), [0.3, 0, 0.5])
        command.mode = Setpoint.MODE_HOVER
        command.position.x = command.position.z = 0.0
        hovered = advance(1, command)
        near(hovered, [0.3, 0, 0.43])
        assert adapter.reference.reference[0] > 0.2
        command.mode = Setpoint.MODE_VELOCITY
        command.velocity.x, command.position.z = 0.1, 0.5
        moved = advance(1, command)
        assert moved[0] > hovered[0] + 0.04, (hovered, moved)
        assert moved[2] > 0.3
        request = Land.Request()
        request.duration = 1.0
        future = land.call_async(request)
        spin_until(future.done)
        assert future.result().success
        assert not adapter.streaming_allowed
        landed = advance(2, command)
        assert landed[2] == pytest.approx(0, abs=0.02)
        print(f'SIL PASS: hover={hovered}, velocity={moved}, land={landed}')
    finally:
        server.visualizations = visuals
        executor.shutdown(timeout_sec=1)
        server.on_shutdown_callback()
        for node in (server, adapter, peer):
            node.destroy_node()
        rclpy.shutdown()
