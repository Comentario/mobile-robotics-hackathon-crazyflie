#!/usr/bin/env python3
"""Verifica el launch instalado sin hardware ni órdenes de vuelo.

Uso: source ROS e install/setup.bash; ROS_DOMAIN_ID=92 python3 scripts/verify_tp12.py
Usar un dominio vacío. Comprueba arranque, API, parámetros, TF y cierre.
"""
import argparse
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from hackathon_interfaces.msg import MultiRange
from hackathon_interfaces.srv import Takeoff, Land
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

EXPECTED = {'simulation_adapter', 'state_monitor', 'range_processor', 'local_world_model',
            'global_planner', 'path_processor', 'local_navigator', 'safety_supervisor',
            'telemetry', 'mission_manager', 'tf_front_range', 'tf_back_range',
            'tf_left_range', 'tf_right_range', 'tf_up_range', 'tf_down_tof'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', default='T01_single_waypoint')
    parser.add_argument('--params', default='simulation.yaml',
                        help='Perfil de parámetros a validar, por ejemplo event.yaml')
    parser.add_argument('--stop-node', action='store_true',
                        help='Termina el simulador para comprobar cierre del stack ante salida de un nodo')
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '92')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/tp12-ros-logs')
    rclpy.init(args=[])
    peer = Node('tp12_bringup_verification')
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    process = None
    log_path = None

    def wait(predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
            if process is not None and process.poll() is not None:
                raise AssertionError(f'Launch terminó antes de tiempo; log: {log_path}')
        raise AssertionError(f'Timeout de verificación; log: {log_path}')

    try:
        # No compartir la prueba con un bringup que ya esté activo.
        for _ in range(10):
            executor.spin_once(timeout_sec=.05)
        assert not EXPECTED.intersection(peer.get_node_names()), 'Usar un dominio ROS vacío'
        with tempfile.TemporaryDirectory(prefix='tp12-') as folder:
            overlay = Path(folder)/'overlay.yaml'
            overlay.write_text('/**:\n  ros__parameters:\n    base_frame: tp12_body\n'
                               'simulation_adapter:\n  ros__parameters:\n'
                               '    range_noise_std: 0.0\n    pose_noise_std: 0.0\n'
                               '    max_speed: 0.4\n')
            log_path = Path(folder)/'launch.log'
            messages = {'odom': [], 'state': [], 'ranges': [], 'tf': [], 'static': []}
            subscriptions = [
                peer.create_subscription(Odometry,'/hk/adapter/odom',messages['odom'].append,10),
                peer.create_subscription(Odometry,'/hk/state',messages['state'].append,10),
                peer.create_subscription(MultiRange,'/hk/adapter/ranges',messages['ranges'].append,10),
                peer.create_subscription(TFMessage,'/tf',messages['tf'].append,10),
                peer.create_subscription(TFMessage,'/tf_static',messages['static'].append,
                    QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            ]
            clients = [peer.create_client(Takeoff,'/hk/takeoff'),
                       peer.create_client(Land,'/hk/land'),
                       peer.create_client(Trigger,'/hk/emergency')]
            with log_path.open('w') as log:
                process = subprocess.Popen(['ros2','launch','hackathon_bringup','simulation.launch.py',
                    f'scenario:={args.scenario}',f'params:={args.params}',f'overlay:={overlay}'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    wait(lambda: EXPECTED.issubset(peer.get_node_names()) and
                         all(messages.values()) and all(c.service_is_ready() for c in clients))
                    assert messages['odom'][-1].child_frame_id == 'tp12_body'
                    assert messages['ranges'][-1].header.frame_id == 'tp12_body'
                    assert messages['state'][-1].header.frame_id == 'map'
                    assert all(math.isfinite(v) or math.isinf(v) for v in
                               (messages['ranges'][-1].front, messages['ranges'][-1].down))
                    frames = {tf.child_frame_id: tf.header.frame_id
                              for msg in messages['static'] for tf in msg.transforms}
                    wait(lambda: len({tf.child_frame_id for msg in messages['static']
                                      for tf in msg.transforms}) >= 6)
                    frames = {tf.child_frame_id: tf.header.frame_id
                              for msg in messages['static'] for tf in msg.transforms}
                    assert all(frames[name] == 'tp12_body' for name in
                               ('front_range','back_range','left_range','right_range','up_range','down_tof'))
                    assert any(tf.child_frame_id == 'tp12_body' and tf.header.frame_id == 'map'
                               for msg in messages['tf'] for tf in msg.transforms)
                    params = peer.create_client(GetParameters,'/simulation_adapter/get_parameters')
                    wait(params.service_is_ready)
                    request = GetParameters.Request()
                    request.names = ['max_speed','pose_noise_std','world_file','scenario_file']
                    future = params.call_async(request)
                    wait(future.done)
                    values = future.result().values
                    assert values[0].double_value == .4 and values[1].double_value == 0
                    assert Path(values[2].string_value).is_file()
                    assert Path(values[3].string_value).is_file()
                    text = log_path.read_text()
                    assert 'Traceback' not in text and 'process has died' not in text
                    print('PASS: 16 nodos, odometría → estado, seis rangos, servicios y TF')
                    print('PASS: mundo/escenario instalados y overlay aplicado a parámetros y frames')
                    if args.stop_node:
                        match = re.search(r'\[sim_node-\d+\]: process started with pid \[(\d+)\]', text)
                        assert match, 'No se encontró el PID del simulador en el log'
                        os.kill(int(match.group(1)), signal.SIGINT)
                        wait(lambda: process.poll() is not None, timeout=15)
                        print('PASS: terminar el simulador cierra el resto del bringup')
                except BaseException:
                    print(log_path.read_text())
                    raise
                finally:
                    if process.poll() is None:
                        # SIGINT al launch equivale al Ctrl+C del usuario.
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=15)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGTERM)
                            process.wait(timeout=5)
                assert process.returncode == 0, log_path.read_text()
                # No esperar mensajes: ya se cerró el productor.
                deadline = time.monotonic()+10
                while EXPECTED.intersection(peer.get_node_names()) and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.1)
                assert not EXPECTED.intersection(peer.get_node_names()), 'Quedaron nodos tras cerrar launch'
                assert 'Traceback' not in log_path.read_text(), log_path.read_text()
                print('PASS: cierre sin nodos residuales' if args.stop_node else
                      'PASS: Ctrl+C cierra launch y todos sus nodos')
    finally:
        executor.shutdown()
        peer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
