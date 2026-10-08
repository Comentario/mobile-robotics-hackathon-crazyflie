"""Coherencia estática de TP-42 C; no certifica hardware ni comportamiento de safety."""
import math
import re


def validate_event(params, hardware, mission):
    errors, pending = [], []
    def section(name):
        return params[name]['ros__parameters']
    common = section('/**')
    adapter, safety = section('cf_adapter'), section('safety_supervisor')
    nav, world = section('local_navigator'), section('local_world_model')
    manager, ranges = section('mission_manager'), section('range_processor')
    def require(condition, message):
        if not condition:
            errors.append(message)
    def positive(*values):
        return all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in values)
    require(positive(common['z_min'], common['z_nominal'], common['z_max']) and
            common['z_min'] < common['z_nominal'] < common['z_max'], 'z_min < z_nominal < z_max, positivos')
    require(positive(common['critical_distance'], common['warning_distance']) and
            common['critical_distance'] < common['warning_distance'], 'critical_distance < warning_distance')
    require(positive(common['state_timeout'], common['range_timeout'], common['ttc_threshold'],
                     safety['command_timeout'], safety['hover_timeout']), 'Timeouts/TTC positivos y finitos')
    require(positive(nav['degraded_speed'], nav['nominal_speed'], nav['max_speed'], nav['max_accel']) and
            nav['degraded_speed'] <= nav['nominal_speed'] <= nav['max_speed'], 'degraded_speed <= nominal_speed <= max_speed')
    require(positive(ranges['range_min'], ranges['range_max']) and ranges['range_min'] < ranges['range_max'], 'range_min < range_max')
    require(positive(world['map_width'], world['map_height'], world['map_resolution'], world['obstacle_inflation_radius']) and
            world['map_resolution'] <= min(world['map_width'], world['map_height']), 'Dimensiones/resolución de mapa válidas')
    require(positive(manager['waypoint_tolerance'], manager['waypoint_timeout'], manager['takeoff_duration'], manager['land_duration']), 'Tolerancia y tiempos de misión positivos')
    require(manager['autostart'] is False, 'La plantilla de evento exige autostart=false')
    require(bool(common['map_frame']) and bool(common['base_frame']) and bool(adapter['source_frame']), 'Frames no vacíos')
    require(mission['frame_id'] == common['map_frame'], 'Frame de misión debe coincidir con map_frame')
    geofence = safety['geofence']
    valid_fence = len(geofence) == 6 and all(math.isfinite(v) for v in geofence) and all(geofence[i] < geofence[i+3] for i in range(3))
    require(valid_fence, 'Geofence [xmin,ymin,zmin,xmax,ymax,zmax] inválido')
    if valid_fence:
        require(geofence[2] <= 0 and common['z_max'] <= geofence[5], 'Geofence debe incluir suelo Z=0 y z_max')
        require(world['map_width'] >= geofence[3]-geofence[0] and world['map_height'] >= geofence[4]-geofence[1], 'Mapa debe cubrir extensión XY de geofence')
    require(bool(mission['waypoints']), 'Misión sin waypoints')
    for waypoint in mission['waypoints']:
        valid = len(waypoint) == 3 and all(math.isfinite(v) for v in waypoint)
        require(valid, 'Waypoint XYZ no finito')
        if valid:
            require(common['z_min'] <= waypoint[2] <= common['z_max'], 'Altura de waypoint fuera de límites')
            if valid_fence:
                require(all(geofence[i] <= waypoint[i] <= geofence[i+3] for i in range(3)), 'Waypoint fuera de geofence')
    name = adapter['robot_name']
    robots = hardware['robots']
    require(name in robots, 'robot_name no coincide con robots del hardware')
    enabled = [key for key, value in robots.items() if value.get('enabled')]
    if enabled:
        require(enabled == [name], 'Habilitar únicamente el robot elegido para esta plantilla')
    else:
        pending.append('Robot deshabilitado en event_hardware.yaml')
    if name in robots:
        robot = robots[name]
        require(robot['type'] in hardware['robot_types'], 'Tipo de robot sin definición')
        require(robot.get('reference_frame') == adapter['source_frame'], 'reference_frame de hardware debe coincidir con source_frame')
        if not re.fullmatch(r'radio://\d+/\d+/(250K|1M|2M)/[0-9a-fA-F]{10}', robot['uri']):
            pending.append('URI de radio pendiente o inválida')
        for value in (name, robot['type']):
            if 'TODO' in value:
                pending.append(f'Identidad/tipo pendiente: {value}')
        logging = robot.get('firmware_logging', {})
        require(logging.get('enabled') is True and all(key in logging.get('default_topics', {}) for key in ('odom','scan')), 'Activar logging odom/scan para la configuración de adapter elegida')
    return errors, pending
