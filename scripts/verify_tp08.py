#!/usr/bin/env python3
"""Exercise installed Crazyswarm2 SIL callbacks and physics, without hardware.

Run after sourcing ROS Humble and the cffirmware Python bindings.
Uses an isolated ROS domain; calls callbacks directly, not ROS service transport.
"""
import argparse
import json
import os
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import numpy as np
import rclpy
import yaml
from crazyflie_interfaces.msg import FullState, Hover
from crazyflie_interfaces.srv import GoTo, Land, NotifySetpointsStop, Takeoff
from crazyflie_sim.crazyflie_server import CrazyflieServer
from std_srvs.srv import Empty


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual', action='store_true', help='Publish TF and open RViz')
    parser.add_argument('--no-rviz', action='store_true', help='With --visual, use an external RViz')
    args = parser.parse_args()
    root = '/opt/ros/humble/share/crazyflie'
    with open(root + '/config/crazyflies.yaml') as f:
        robots = yaml.safe_load(f)
    with open(root + '/config/server.yaml') as f:
        params = yaml.safe_load(f)['/crazyflie_server']['ros__parameters']
    params.update(robots)
    with open('/opt/ros/humble/share/crazyflie_description/urdf/crazyflie_description.urdf') as f:
        params['robot_description'] = f.read()
    for vis in params['sim']['visualizations'].values():
        vis['enabled'] = False
    params['sim']['visualizations']['rviz']['enabled'] = args.visual
    assert [name for name, cfg in robots['robots'].items() if cfg['enabled']] == ['cf231']
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as config:
        yaml.safe_dump({'/crazyflie_server': {'ros__parameters': params}}, config)
        config.flush()
        rclpy.init(args=['--ros-args', '--params-file', config.name])
        server = CrazyflieServer()
    results = {}
    viewer = None
    view_config = None
    wall_start = time.monotonic()
    visualizations = server.visualizations
    ticks = 0

    def advance(seconds):
        nonlocal ticks
        for _ in range(round(seconds / server.backend.dt)):
            # Physics stays at 2 kHz; TF and visual pacing run at 50 Hz.
            server.visualizations = visualizations if ticks % 40 == 0 else []
            server._timer_callback()
            ticks += 1
            if args.visual and ticks % 40 == 0:
                delay = wall_start + server.backend.time() - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        pos = server.backend.uavs[0].state.pos.tolist()
        assert np.isfinite(pos).all(), pos
        return pos

    def near(pos, target, tolerance=0.12):
        assert np.linalg.norm(np.array(pos) - target) < tolerance, (pos, target)

    try:
        if args.visual:
            if not args.no_rviz:
                with open(root + '/config/config.rviz') as f:
                    config = yaml.safe_load(f)
                for display in config['Visualization Manager']['Displays']:
                    if display['Class'] == 'rviz_default_plugins/RobotModel':
                        display['Description Topic']['Durability Policy'] = 'Transient Local'
                view_config = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz')
                yaml.safe_dump(config, view_config)
                view_config.flush()
                viewer = subprocess.Popen([
                    'rviz2', '-d', view_config.name,
                    '--ros-args', '-p', 'use_sim_time:=true',
                ])
            print('Modo visual: ROS_DOMAIN_ID=88. Preparando RViz durante 5 segundos...', flush=True)
            advance(0.1)
            time.sleep(5)
            if viewer is not None and viewer.poll() is not None:
                raise RuntimeError('RViz no pudo abrirse; revisa DISPLAY y el soporte gráfico de Ubuntu')
            wall_start = time.monotonic() - server.backend.time()
        print('1. Despegue a 0.5 m', flush=True)
        request = Takeoff.Request()
        request.height = 0.5
        request.duration.sec = 2
        server._takeoff_callback(request, Takeoff.Response(), name='cf231')
        results['takeoff'] = advance(3)
        near(results['takeoff'], [0, 0, 0.5])

        print('2. go_to: X=0.3 m, Z=0.5 m', flush=True)
        request = GoTo.Request()
        request.goal.x, request.goal.z = 0.3, 0.5
        request.duration.sec = 2
        server._go_to_callback(request, GoTo.Response(), name='cf231')
        results['go_to'] = advance(3)
        near(results['go_to'], [0.3, 0, 0.5])

        print('3. FullState: X=0.4 m, Z=0.5 m, a 50 Hz', flush=True)
        command = FullState()
        command.pose.position.x, command.pose.position.z = 0.4, 0.5
        command.pose.orientation.w = 1.0
        for _ in range(100):
            server._cmd_full_state_changed(command, name='cf231')
            advance(0.02)
        results['full_state_stream'] = advance(0.1)
        near(results['full_state_stream'], [0.4, 0, 0.5])
        print('4. Interrupción de comandos durante 3 segundos simulados', flush=True)
        results['stream_stopped_3s'] = advance(3)
        near(results['stream_stopped_3s'], [0.4, 0, 0.5])

        print('5. Callbacks sin implementar y transición go_to rechazada', flush=True)
        mode = server.cfs['cf231'].mode
        server._notify_setpoints_stop_callback(
            NotifySetpointsStop.Request(), NotifySetpointsStop.Response(), name='cf231')
        server._cmd_hover_changed(Hover(), name='cf231')
        server._emergency_callback(Empty.Request(), Empty.Response(), name='cf231')
        assert server.cfs['cf231'].mode == mode
        results['unimplemented_callbacks'] = 'mode unchanged; see server logs'
        try:
            server._go_to_callback(request, GoTo.Response(), name='cf231')
        except ValueError as exc:
            results['go_to_after_stream'] = str(exc)
        else:
            raise AssertionError('Expected SIL to reject go_to after streaming')

        print('6. Aterrizaje', flush=True)
        request = Land.Request()
        request.height = 0.0
        request.duration.sec = 2
        server._land_callback(request, Land.Response(), name='cf231')
        results['land_after_stream'] = advance(3)
        near(results['land_after_stream'], [0.4, 0, 0], tolerance=0.15)
        print(json.dumps({'status': 'PASS', 'results': results}, indent=2))
        if viewer is not None:
            input('Secuencia completada. Pulsa Enter para cerrar RViz y el simulador. ')
    finally:
        server.visualizations = visualizations
        server.on_shutdown_callback()
        server.destroy_node()
        rclpy.shutdown()
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
            try:
                viewer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                viewer.kill()
                viewer.wait()
        if view_config is not None:
            view_config.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
