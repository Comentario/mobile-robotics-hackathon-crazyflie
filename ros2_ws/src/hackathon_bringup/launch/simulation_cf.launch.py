"""MVP integrado con la API de persona 2.

backend:=emulator usa sim_crazyflie (cinemático propio, portable).
backend:=sil conecta cf_adapter + sim_sensors a Crazyswarm2 externo, backend sim.
Para sil todos los nodos usan /clock. No inicia radio ni hardware.
"""
import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from hackathon_bringup.stack import config_file,param_files,stack_nodes,static_sensor_tfs,shared_frames,stop_on_exit


def setup(context):
    config=os.path.join(get_package_share_directory('hackathon_bringup'),'config')
    arg=lambda name: LaunchConfiguration(name).perform(context)
    scenario_path=config_file(config,'scenarios/'+arg('scenario')+'.yaml')
    with open(scenario_path,encoding='utf-8') as stream: scenario=yaml.safe_load(stream)
    params=param_files(config,arg('params'),arg('overlay'))
    backend=arg('backend')
    if backend not in ('emulator','sil'): raise ValueError('backend debe ser emulator o sil')
    robot=arg('robot_name')
    # Estos parámetros de launch prevalecen sobre overlays antiguos incompatibles.
    common={'use_sim_time':backend=='sil','scenario_name':arg('scenario')}
    source={'robot_name':robot,'scenario_file':scenario_path,
            'world_file':config_file(config,scenario['world'])}
    # ROS da prioridad a secciones por nodo frente a /**, incluso si el
    # wildcard se pasa después. Materializar overrides por nodo evita eso.
    merged={}
    for path in params:
        with open(path,encoding='utf-8') as stream:
            for key,section in (yaml.safe_load(stream) or {}).items():
                merged.setdefault(key,{'ros__parameters':{}})['ros__parameters'].update(section.get('ros__parameters',{}))
    for name in ['cf_adapter','sim_crazyflie','sim_sensors','mission_manager','state_monitor','range_processor','local_navigator','safety_supervisor','local_world_model','global_planner','path_processor','telemetry']:
        merged.setdefault(name,{'ros__parameters':{}})['ros__parameters'].update(common)
    merged['cf_adapter']['ros__parameters'].update({'robot_name':robot,'state_source':'pose',
        'pose_topic':f'/{robot}/pose','scan_topic':f'/{robot}/scan',
        'vertical_ranges_topic':f'/{robot}/ranges_extra','source_frame':'world'})
    for name in ('sim_crazyflie','sim_sensors'):
        merged[name]['ros__parameters'].update(source)
    with tempfile.NamedTemporaryFile(mode='w',prefix='hk-integrated-',suffix='.yaml',delete=False) as stream:
        yaml.safe_dump(merged,stream); resolved=stream.name
    params=[resolved]
    nodes=[Node(package='simulation_adapter',executable='sim_sensors' if backend=='sil' else 'sim_crazyflie',
                name='sim_sensors' if backend=='sil' else 'sim_crazyflie',output='screen',
                parameters=params+[common,source]),
           Node(package='cf_adapter',executable='cf_adapter_node',name='cf_adapter',output='screen',
                parameters=params+[common,{'robot_name':robot,'state_source':'pose',
                  'pose_topic':f'/{robot}/pose','scan_topic':f'/{robot}/scan',
                  'vertical_ranges_topic':f'/{robot}/ranges_extra','source_frame':'world'}])]
    nodes+=stack_nodes(params,config_file(config,scenario['mission']),common)
    nodes+=static_sensor_tfs(config,shared_frames(params))
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    def remove_config(context):
        if os.path.exists(resolved):
            os.unlink(resolved)
        return []
    cleanup=RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=remove_config)]))
    return stop_on_exit(nodes)+nodes+[cleanup]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend',default_value='emulator'),
        DeclareLaunchArgument('scenario',default_value='T01_single_waypoint'),
        DeclareLaunchArgument('params',default_value='simulation.yaml'),
        DeclareLaunchArgument('overlay',default_value=''),
        DeclareLaunchArgument('robot_name',default_value='cf231'),
        OpaqueFunction(function=setup)])
