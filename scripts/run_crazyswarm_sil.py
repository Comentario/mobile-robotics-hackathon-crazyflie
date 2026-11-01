#!/usr/bin/env python3
"""Starts the installed Crazyswarm2 SIL, without radio or a graphical window.

Requires crazyflie_sim and cffirmware bindings compatible with that installation.
Publishes /clock and TF world -> cf231; sim_sensors generates the ToF separately.
"""
import argparse
from pathlib import Path
import tempfile
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from crazyflie_sim.crazyflie_server import CrazyflieServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', default='T01_single_waypoint')
    args = parser.parse_args()
    config = Path(get_package_share_directory('crazyflie')) / 'config'
    project = Path(get_package_share_directory('hackathon_bringup')) / 'config'
    robots = yaml.safe_load((config / 'crazyflies.yaml').read_text())
    scenario = yaml.safe_load((project / 'scenarios' / (args.scenario + '.yaml')).read_text())
    for name, robot in robots['robots'].items():
        robot['enabled'] = name == 'cf231'
        if robot['enabled']:
            robot['initial_position'] = scenario['start'][:3]
    if 'cf231' not in robots['robots']:
        raise ValueError('The Crazyswarm2 configuration must define cf231')
    params = yaml.safe_load((config / 'server.yaml').read_text())['/crazyflie_server']['ros__parameters']
    params.update(robots)
    description = Path(get_package_share_directory('crazyflie_description')) / 'urdf' / 'crazyflie_description.urdf'
    params['robot_description'] = description.read_text()
    params['sim']['backend'] = 'np'
    params['sim']['max_dt'] = 0.0
    for name, visualization in params['sim']['visualizations'].items():
        visualization['enabled'] = name == 'rviz'  # This module only publishes TF.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as stream:
        yaml.safe_dump({'/crazyflie_server': {'ros__parameters': params}}, stream)
        stream.flush()
        rclpy.init(args=['--ros-args', '--params-file', stream.name])
        server = CrazyflieServer()
    # Physics still runs at 2 kHz; publishing TF at 50 Hz avoids saturating the ROS nodes.
    for visualization in server.visualizations:
        original_step = visualization.step
        previous = [-1.0]
        def publish_tf(t, states, desired, actions, step=original_step, last=previous):
            if t-last[0] >= .02:
                step(t, states, desired, actions)
                last[0] = t
        visualization.step = publish_tf
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.on_shutdown_callback()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
