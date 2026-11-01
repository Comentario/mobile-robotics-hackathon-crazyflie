"""Nodes shared by simulation.launch.py and real.launch.py.

VS-5: between simulation and hardware only the adapter and the configuration change.
"""
import os

import yaml
from launch_ros.actions import Node

# Package = node name = section in the parameter YAML files.
STACK = [
    ('state_monitor', 'state_monitor_node'),
    ('range_processor', 'range_processor_node'),
    ('local_world_model', 'local_world_model_node'),
    ('global_planner', 'global_planner_node'),
    ('path_processor', 'path_processor_node'),
    ('local_navigator', 'local_navigator_node'),
    ('safety_supervisor', 'safety_supervisor_node'),
    ('telemetry', 'telemetry_node'),
]


def param_files(config_dir, params, overlay):
    """Base file + optional overlay (e.g. conservative.yaml); the overlay wins."""
    files = [config_file(config_dir, params)]
    if overlay:
        files.append(config_file(config_dir, overlay))
    return files


def static_sensor_tfs(config_dir, base_frame=None):
    """static_transform_publisher base_link -> frame of every sensor (config/frames.yaml)."""
    with open(os.path.join(config_dir, 'frames.yaml'), encoding='utf-8') as f:
        frames = yaml.safe_load(f)
    nodes = []
    for child, tf in frames['sensors'].items():
        x, y, z = (str(v) for v in tf['xyz'])
        roll, pitch, yaw = (str(v) for v in tf['rpy'])
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_{child}',
            arguments=['--x', x, '--y', y, '--z', z,
                       '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                       '--frame-id', base_frame or frames['base_frame'], '--child-frame-id', child],
        ))
    return nodes


def stack_nodes(params, mission_file, extra=None, skip=()):
    """Stack nodes. `extra`: additional parameters for all of them (e.g. scenario_name).

    `skip`: STACK packages that are not launched (e.g. the planner in cooperative route mode).
    """
    extra = extra or {}
    nodes = [
        Node(package=pkg, executable=exe, name=pkg, output='screen',
             parameters=params + [extra])
        for pkg, exe in STACK
        if pkg not in skip
    ]
    nodes.append(Node(
        package='mission_manager', executable='mission_manager_node', name='mission_manager',
        output='screen', parameters=params + [extra, {'mission_file': mission_file}]))
    return nodes


def config_file(config_dir, filename):
    """Accept a path absolute or relative to config and require it to exist."""
    path = os.path.abspath(os.path.join(config_dir, filename))
    if not os.path.isfile(path):
        raise ValueError(f'Configuration file does not exist: {path}')
    return path


def shared_frames(params):
    """Honour map_frame/base_frame overlays in the static TFs too."""
    shared = {}
    for path in params:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        shared.update(data.get('/**', {}).get('ros__parameters', {}))
    return shared.get('base_frame')


def stop_on_exit(nodes):
    """A node that exits shuts the bringup down so no partial stack is left running."""
    from launch.actions import RegisterEventHandler, Shutdown
    from launch.event_handlers import OnProcessExit
    return [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[Shutdown(reason='A bringup node exited')]))
        for node in nodes]
