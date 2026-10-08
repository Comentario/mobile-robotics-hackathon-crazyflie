"""Stack completo + cf_adapter + dron simulado con la API de Crazyswarm2 (TP-10, TP-09).

Misma cadena que con el dron real (ver docs/sensor_model.md, diagrama 1 y 2):

  simulación:  stack -> cf_adapter -> sim_crazyflie (dron + mundo simulados)
  Dresden:     stack -> cf_adapter -> crazyflie_server -> Crazyradio -> dron real

Por eso aquí NO se arranca simulation_adapter/sim_node (ese publicaba /hk/adapter/*
directamente y se saltaba cf_adapter).

Uso (igual que simulation.launch.py):
  ros2 launch hackathon_bringup simulation_cf.launch.py scenario:=T03_static_wall
  ros2 launch hackathon_bringup simulation_cf.launch.py scenario:=T08_range_spike rviz:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hackathon_bringup.stack import param_files, stack_nodes, static_sensor_tfs


def _setup(context):
    share = get_package_share_directory('hackathon_bringup')
    config_dir = os.path.join(share, 'config')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    scenario_name = arg('scenario')
    scenario_file = os.path.join(config_dir, 'scenarios', scenario_name + '.yaml')
    with open(scenario_file, encoding='utf-8') as f:
        scenario = yaml.safe_load(f)

    params = param_files(config_dir, arg('params'), arg('overlay'))
    extra = {'scenario_name': scenario_name}

    nodes = [
        # Bloque que en Dresden se cambia por crazyflie_server + Crazyradio + dron.
        Node(
            package='simulation_adapter', executable='sim_crazyflie', name='sim_crazyflie',
            output='screen',
            parameters=params + [extra, {
                'robot_name': arg('robot_name'),
                'world_file': os.path.join(config_dir, scenario['world']),
                'scenario_file': scenario_file,
            }],
        ),
        # El MISMO cf_adapter que en Dresden.
        Node(
            package='cf_adapter', executable='cf_adapter_node', name='cf_adapter',
            output='screen',
            parameters=params + [extra, {
                'robot_name': arg('robot_name'),
                'pose_source': 'topic',
            }],
        ),
    ]
    nodes += static_sensor_tfs(config_dir)
    nodes += stack_nodes(params, os.path.join(config_dir, scenario['mission']), extra)

    if arg('evaluate') == 'true':
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

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='T01_single_waypoint',
                              description='Nombre de config/scenarios/<scenario>.yaml'),
        DeclareLaunchArgument('params', default_value='simulation.yaml',
                              description='Parámetros base en config/'),
        DeclareLaunchArgument('overlay', default_value='',
                              description='Parámetros que sobrescriben la base'),
        DeclareLaunchArgument('robot_name', default_value='cf231',
                              description='Nombre del dron (como en crazyflies.yaml)'),
        DeclareLaunchArgument('evaluate', default_value='false',
                              description='true: lanza hackathon_tests/run_scenario (pass/fail)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
