#!/usr/bin/env python3
"""T01/T02 integrados: API persona 2 + misión/seguidor/safety reales del equipo.

Ejecutar con ROS e install/setup.bash cargados. No radio. Para sil requiere
crazyflie_server backend sim ya en marcha, con /clock y TF world→cf231.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import FullState,LogDataGeneric
from hackathon_interfaces.msg import MissionStatus,SafetyState,Setpoint,MultiRange,LocalizationHealth


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',default='T01_single_waypoint')
    parser.add_argument('--backend',choices=['emulator','sil'],default='emulator')
    parser.add_argument('--timeout',type=float,default=90)
    args=parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID','93')
    os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
    os.environ.setdefault('ROS_LOG_DIR','/tmp/hk-mvp-ros-logs')
    rclpy.init(args=['--ros-args','-p',f'use_sim_time:={str(args.backend=="sil").lower()}'])
    peer=Node('mvp_integration_verifier');executor=SingleThreadedExecutor();executor.add_node(peer)
    messages={name:[] for name in ('pose','scan','extra','full','state','raw_ranges','ranges','rates','health','safe','nav','safety','mission')}
    topics=[('pose','/cf231/pose',PoseStamped),('scan','/cf231/scan',LaserScan),
            ('extra','/cf231/ranges_extra',LogDataGeneric),('full','/cf231/cmd_full_state',FullState),
            ('state','/hk/state',Odometry),('raw_ranges','/hk/adapter/ranges',MultiRange),
            ('ranges','/hk/ranges',MultiRange),('rates','/hk/ranges/closing_rate',MultiRange),
            ('health','/hk/localization_health',LocalizationHealth),('safe','/hk/cmd/safe',Setpoint),
            ('nav','/hk/cmd/nav',Setpoint),('safety','/hk/safety_state',SafetyState),
            ('mission','/hk/mission_status',MissionStatus)]
    subscriptions=[peer.create_subscription(kind,topic,messages[key].append,100) for key,topic,kind in topics]
    process=None
    def wait(predicate,timeout=30):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
            if process and process.poll() is not None: raise AssertionError('Launch terminó antes de tiempo')
        raise AssertionError('Timeout; revisar salud, safety y logs de integración')
    try:
        with tempfile.TemporaryDirectory(prefix='hk-mvp-') as folder:
            overlay=Path(folder)/'test.yaml'
            overlay.write_text('mission_manager:\n  ros__parameters:\n    autostart: false\n'
                               'sim_crazyflie:\n  ros__parameters:\n    pose_noise_std: 0.0\n    range_noise_std: 0.0\n'
                               'sim_sensors:\n  ros__parameters:\n    pose_noise_std: 0.0\n    range_noise_std: 0.0\n')
            log_path=Path('/tmp')/f'hk_mvp_{args.backend}_{args.scenario}.log'
            with log_path.open('w') as log:
                process=subprocess.Popen(['ros2','launch','hackathon_bringup','simulation_cf.launch.py',
                    f'backend:={args.backend}',f'scenario:={args.scenario}',f'overlay:={overlay}'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    wait(lambda: all(messages[k] for k in ('pose','scan','extra','state','ranges','rates','health','safety','mission')))
                    wait(lambda: messages['health'][-1].status==LocalizationHealth.GOOD and
                         messages['safety'][-1].state==SafetyState.NORMAL)
                    assert peer.count_publishers('/cf231/pose')==1
                    assert peer.count_publishers('/cf231/scan')==1
                    assert peer.count_publishers('/hk/cmd/safe')==1
                    assert peer.count_publishers('/hk/adapter/odom')==1
                    assert peer.count_publishers('/hk/adapter/ranges')==1
                    start=peer.create_client(Trigger,'/hk/mission/start');wait(start.service_is_ready)
                    future=start.call_async(Trigger.Request());wait(future.done)
                    assert future.result().success,future.result().message
                    wait(lambda: messages['mission'][-1].state in (MissionStatus.COMPLETED,MissionStatus.ABORTED),args.timeout)
                    assert messages['mission'][-1].state==MissionStatus.COMPLETED,messages['mission'][-1].reason
                    wait(lambda: abs(messages['state'][-1].pose.pose.position.z)<.025,15)
                    assert all(messages[k] for k in messages),[k for k,v in messages.items() if not v]
                    assert any(msg.mode==Setpoint.MODE_VELOCITY for msg in messages['nav'])
                    assert all(msg.header.frame_id=='map' for msg in messages['safe'])
                    assert messages['ranges'][-1].header.frame_id=='base_link'
                    assert 'Traceback' not in log_path.read_text(),log_path.read_text()
                    print(f'PASS {args.scenario} ({args.backend}): pose → salud/rangos → misión → navegación → safety → cf_adapter → dron → aterrizaje')
                    print(f'PASS topics acordados y publisher único; log: {log_path}')
                except BaseException:
                    if messages['state']:
                        print('Última posición:', messages['state'][-1].pose.pose.position)
                    for key in ('ranges','rates','safety'):
                        if messages[key]: print(key, messages[key][-1])
                    print(log_path.read_text()[-15000:])
                    raise
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try: process.wait(timeout=15)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                    assert process.returncode==0,log_path.read_text()[-5000:]
    finally:
        executor.shutdown();peer.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
