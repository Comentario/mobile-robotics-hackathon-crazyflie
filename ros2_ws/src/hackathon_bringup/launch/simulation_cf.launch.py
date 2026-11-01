"""Integrated MVP with the Crazyswarm2 API of person 2 (cf_adapter path, as on hardware).

backend:=emulator uses sim_crazyflie (our own portable kinematic model).
backend:=sil connects cf_adapter + sim_sensors to an external Crazyswarm2 sim backend.
With sil every node uses /clock. Starts neither radio nor hardware.

Cooperative CF2 -> CF1 route: route:=<route.json> (or the scenario `route` key)
replaces global_planner/path_processor with scripts/cooperative_route.py and enables
use_planned_path in local_navigator, at the navigation height of the route.
rviz:=true shows the CF2 map, the route and the flown trajectory (scripts/route_view.py).
"""
import json
import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from hackathon_bringup.stack import config_file,param_files,stack_nodes,static_sensor_tfs,shared_frames,stop_on_exit


def repo_root():
    """Repository root from ros2_ws/install/hackathon_bringup/share/hackathon_bringup."""
    share=get_package_share_directory('hackathon_bringup')
    return os.path.normpath(os.path.join(share,*['..']*5))


def setup(context):
    config=os.path.join(get_package_share_directory('hackathon_bringup'),'config')
    arg=lambda name: LaunchConfiguration(name).perform(context)
    scenario_path=config_file(config,'scenarios/'+arg('scenario')+'.yaml')
    with open(scenario_path,encoding='utf-8') as stream: scenario=yaml.safe_load(stream)
    route_path=arg('route') or scenario.get('route','')
    route=None
    if route_path:
        if not os.path.isabs(route_path):
            route_path=os.path.join(repo_root(),route_path)
        with open(route_path,encoding='utf-8') as stream: route=json.load(stream)
    params=param_files(config,arg('params'),arg('overlay'))
    backend=arg('backend')
    if backend not in ('emulator','sil'): raise ValueError('backend must be emulator or sil')
    robot=arg('robot_name')
    # These launch parameters take precedence over old, incompatible overlays.
    common={'use_sim_time':backend=='sil','scenario_name':arg('scenario')}
    source={'robot_name':robot,'scenario_file':scenario_path,
            'world_file':config_file(config,scenario['world'])}
    # ROS gives per-node sections priority over /**, even if the wildcard is
    # passed later. Materializing per-node overrides avoids that.
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
    if route is not None:
        merged['local_navigator']['ros__parameters'].update({
            'use_planned_path':True,'z_nominal':float(route['navigation_height_m'])})
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
    skip=('global_planner','path_processor') if route is not None else ()
    nodes+=stack_nodes(params,config_file(config,scenario['mission']),common,skip)
    if route is not None:
        server=arg('route_server') or os.path.join(repo_root(),'scripts','cooperative_route.py')
        if not os.path.isfile(server): raise FileNotFoundError(f'No existe {server}; usar route_server:=')
        cmd=['python3',server,'--route',route_path]+(['--allow-synthetic'] if route.get('simulation') else [])
        nodes.append(ExecuteProcess(cmd=cmd,name='cooperative_route',output='screen'))
    if arg('rviz')=='true':
        # CF2 map, planned route, current leg and flown trajectory; closing RViz stops everything.
        view=['python3',os.path.join(repo_root(),'scripts','route_view.py')]+(['--route',route_path] if route is not None else [])
        nodes.append(ExecuteProcess(cmd=view,name='route_view',output='screen'))
        rviz_config=os.path.join(get_package_share_directory('hackathon_bringup'),'rviz','cf1_route.rviz')
        # Software rendering: avoids GPU/shader problems in WSLg.
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',rviz_config],
                          additional_env={'LIBGL_ALWAYS_SOFTWARE':'1','QT_QPA_PLATFORM':'xcb'}))
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
        DeclareLaunchArgument('route',default_value='',
                              description='Cooperative route JSON (plan_shared_map / generate_polygon_preview)'),
        DeclareLaunchArgument('route_server',default_value='',
                              description='Path to cooperative_route.py if it is not in <repo>/scripts'),
        DeclareLaunchArgument('rviz',default_value='false',
                              description='true: RViz with the CF2 map, route and flown trajectory'),
        OpaqueFunction(function=setup)])
