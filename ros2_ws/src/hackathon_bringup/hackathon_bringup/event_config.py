"""Static TP-42 C consistency check; it certifies neither hardware nor safety behaviour."""
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
            common['z_min'] < common['z_nominal'] < common['z_max'], 'z_min < z_nominal < z_max, positive')
    require(positive(common['critical_distance'], common['warning_distance']) and
            common['critical_distance'] < common['warning_distance'], 'critical_distance < warning_distance')
    require(positive(common['state_timeout'], common['range_timeout'], common['ttc_threshold'],
                     safety['command_timeout'], safety['hover_timeout']), 'Timeouts/TTC positive and finite')
    require(positive(nav['degraded_speed'], nav['nominal_speed'], nav['max_speed'], nav['max_accel']) and
            nav['degraded_speed'] <= nav['nominal_speed'] <= nav['max_speed'], 'degraded_speed <= nominal_speed <= max_speed')
    require(positive(ranges['range_min'], ranges['range_max']) and ranges['range_min'] < ranges['range_max'], 'range_min < range_max')
    require(positive(world['map_width'], world['map_height'], world['map_resolution'], world['obstacle_inflation_radius']) and
            world['map_resolution'] <= min(world['map_width'], world['map_height']), 'Valid map dimensions/resolution')
    require(positive(manager['waypoint_tolerance'], manager['waypoint_timeout'], manager['takeoff_duration'], manager['land_duration']), 'Positive mission tolerance and times')
    require(manager['autostart'] is False, 'The event template requires autostart=false')
    require(bool(common['map_frame']) and bool(common['base_frame']) and bool(adapter['source_frame']), 'Non-empty frames')
    require(mission['frame_id'] == common['map_frame'], 'Mission frame must match map_frame')
    geofence = safety['geofence']
    valid_fence = len(geofence) == 6 and all(math.isfinite(v) for v in geofence) and all(geofence[i] < geofence[i+3] for i in range(3))
    require(valid_fence, 'Invalid geofence [xmin,ymin,zmin,xmax,ymax,zmax]')
    if valid_fence:
        require(geofence[2] <= 0 and common['z_max'] <= geofence[5], 'Geofence must include the floor Z=0 and z_max')
        require(world['map_width'] >= geofence[3]-geofence[0] and world['map_height'] >= geofence[4]-geofence[1], 'Map must cover the XY extent of the geofence')
    require(bool(mission['waypoints']), 'Mission without waypoints')
    for waypoint in mission['waypoints']:
        valid = len(waypoint) == 3 and all(math.isfinite(v) for v in waypoint)
        require(valid, 'Non-finite waypoint XYZ')
        if valid:
            require(common['z_min'] <= waypoint[2] <= common['z_max'], 'Waypoint height out of limits')
            if valid_fence:
                require(all(geofence[i] <= waypoint[i] <= geofence[i+3] for i in range(3)), 'Waypoint outside the geofence')
    name = adapter['robot_name']
    robots = hardware['robots']
    require(name in robots, 'robot_name does not match the hardware robots')
    enabled = [key for key, value in robots.items() if value.get('enabled')]
    if enabled:
        require(enabled == [name], 'Enable only the robot chosen for this template')
    else:
        pending.append('Robot disabled in event_hardware.yaml')
    if name in robots:
        robot = robots[name]
        require(robot['type'] in hardware['robot_types'], 'Robot type without definition')
        require(robot.get('reference_frame') == adapter['source_frame'], 'Hardware reference_frame must match source_frame')
        if not re.fullmatch(r'radio://\d+/\d+/(250K|1M|2M)/[0-9a-fA-F]{10}', robot['uri']):
            pending.append('Radio URI pending or invalid')
        for value in (name, robot['type']):
            if 'TODO' in value:
                pending.append(f'Identity/type pending: {value}')
        logging = robot.get('firmware_logging', {})
        require(logging.get('enabled') is True and all(key in logging.get('default_topics', {}) for key in ('odom','scan')), 'Enable odom/scan logging for the chosen adapter configuration')
    return errors, pending
