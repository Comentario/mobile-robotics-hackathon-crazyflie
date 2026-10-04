"""Nodos comunes a simulation.launch.py y real.launch.py.

VS-5: entre simulación y hardware solo cambia el adapter y la configuración.
"""
import os

import yaml
from launch_ros.actions import Node

# Paquete = nombre del nodo = sección en los YAML de parámetros.
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
    """Fichero base + overlay opcional (p. ej. conservative.yaml); el overlay gana."""
    files = [os.path.join(config_dir, params)]
    if overlay:
        files.append(os.path.join(config_dir, overlay))
    return files


def static_sensor_tfs(config_dir):
    """static_transform_publisher base_link -> frame de cada sensor (config/frames.yaml)."""
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
                       '--frame-id', frames['base_frame'], '--child-frame-id', child],
        ))
    return nodes


def stack_nodes(params, mission_file, extra=None):
    """Nodos del stack. `extra`: parámetros adicionales para todos (p. ej. scenario_name)."""
    extra = extra or {}
    nodes = [
        Node(package=pkg, executable=exe, name=pkg, output='screen',
             parameters=params + [extra])
        for pkg, exe in STACK
    ]
    nodes.append(Node(
        package='mission_manager', executable='mission_manager_node', name='mission_manager',
        output='screen', parameters=params + [extra, {'mission_file': mission_file}]))
    return nodes
