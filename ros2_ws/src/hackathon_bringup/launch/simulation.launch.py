"""Stack completo contra el simulador propio (TP-12, VS-1 a VS-4).

Uso:
  ros2 launch hackathon_bringup simulation.launch.py
  ros2 launch hackathon_bringup simulation.launch.py scenario:=T05_popup_obstacle
  ros2 launch hackathon_bringup simulation.launch.py scenario:=T09_range_dropout evaluate:=true
  ros2 launch hackathon_bringup simulation.launch.py overlay:=conservative.yaml rviz:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hackathon_bringup.stack import (param_files, stack_nodes, static_sensor_tfs,
                                    config_file, shared_frames, stop_on_exit)


def _setup(context):
    share = get_package_share_directory('hackathon_bringup')
    config_dir = os.path.join(share, 'config')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    scenario_name = arg('scenario')
    scenario_file = config_file(config_dir, os.path.join('scenarios', scenario_name + '.yaml'))
    with open(scenario_file, encoding='utf-8') as f:
        scenario = yaml.safe_load(f)

    params = param_files(config_dir, arg('params'), arg('overlay'))
    world_file = config_file(config_dir, scenario['world'])
    mission_file = config_file(config_dir, scenario['mission'])
    extra = {'scenario_name': scenario_name}

    nodes = [Node(
        package='simulation_adapter', executable='sim_node', name='simulation_adapter',
        output='screen',
        parameters=params + [extra, {
            'world_file': world_file,
            'scenario_file': scenario_file,
            'publish_tf': False, # state_monitor es el único propietario de map → base_link
        }],
    )]
    nodes += static_sensor_tfs(config_dir, shared_frames(params))
    nodes += stack_nodes(params, mission_file, extra)

    if arg('evaluate') == 'true':
        # Al terminar la evaluación se cierra todo el launch (necesario para run_all_scenarios.sh).
        runner = Node(
            package='hackathon_tests', executable='run_scenario', name='scenario_runner',
            output='screen', parameters=params + [extra, {'scenario_file': scenario_file}])
        nodes.append(runner)
        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[Shutdown(reason='scenario evaluated')])))

    if arg('rviz') == 'true':
        rviz_config = os.path.join(share, 'rviz', 'hackathon.rviz')
        rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=rviz_args))

    processes = [node for node in nodes if isinstance(node, Node)]
    return stop_on_exit(processes) + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='T01_single_waypoint',
                              description='Nombre de config/scenarios/<scenario>.yaml'),
        DeclareLaunchArgument('params', default_value='simulation.yaml',
                              description='Parámetros base en config/'),
        DeclareLaunchArgument('overlay', default_value='',
                              description='Parámetros que sobrescriben la base, p. ej. conservative.yaml'),
        DeclareLaunchArgument('evaluate', default_value='false',
                              description='true: lanza hackathon_tests/run_scenario (pass/fail)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
