"""Full stack against Crazyswarm2 (real hardware or the Crazyswarm2 sim backend). TP-12, TP-42, VS-5.

Requires the Crazyswarm2 server running, in another terminal:
  ros2 launch crazyflie launch.py backend:=cflib     # real hardware
  ros2 launch crazyflie launch.py backend:=sim       # simulation level 2 (no ranges)

Usage:
  ros2 launch hackathon_bringup real.launch.py
  ros2 launch hackathon_bringup real.launch.py params:=simulation.yaml overlay:=cs2_sim.yaml mission:=missions/single.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hackathon_bringup.stack import (param_files, stack_nodes, static_sensor_tfs,
                                    config_file, shared_frames, stop_on_exit)


def _setup(context):
    share = get_package_share_directory('hackathon_bringup')
    config_dir = os.path.join(share, 'config')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    params = param_files(config_dir, arg('params'), arg('overlay'))
    nodes = [Node(package='cf_adapter', executable='cf_adapter_node', name='cf_adapter',
                  output='screen', parameters=params)]
    nodes += static_sensor_tfs(config_dir, shared_frames(params))
    nodes += stack_nodes(params, config_file(config_dir, arg('mission')))
    return stop_on_exit(nodes) + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='event.yaml'),
        DeclareLaunchArgument('overlay', default_value=''),
        DeclareLaunchArgument('mission', default_value='missions/event.yaml'),
        OpaqueFunction(function=_setup),
    ])
