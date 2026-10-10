"""Read telemetry, or explicitly request a short Flow-deck hover.

Uses Bitcraze's SyncCrazyflie, LogConfig and MotionCommander APIs.
No commands are sent to the motors without --fly.
"""
import argparse
import json
import time
import math
import socket
from uuid import uuid4
from pathlib import Path
from threading import Event
from threading import Lock
from contextlib import ExitStack
from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


def read_supervisor(cf):
    """Read a fresh firmware log value, including on legacy CRTP firmware."""
    if 'info' not in cf.log.toc.toc.get('supervisor', {}):
        raise RuntimeError('Firmware does not expose supervisor.info; cannot verify arming state')
    ready = Event()
    result = []
    config = LogConfig(name='cf2_supervisor_check', period_in_ms=100)
    config.add_variable('supervisor.info')

    def received(timestamp, data, logconf):
        result.append(int(data['supervisor.info']))
        ready.set()

    config.data_received_cb.add_callback(received)
    cf.log.add_config(config)
    try:
        config.start()
        if not ready.wait(3):
            raise RuntimeError('No fresh supervisor state; no takeoff')
        return result[-1]
    finally:
        config.delete()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uri', required=True)
    parser.add_argument('--fly', action='store_true')
    parser.add_argument('--navigate', action='store_true',
                        help='Closed-loop TP navigation/avoidance; without a route, reactive hover')
    parser.add_argument('--speed', type=float, default=0.1)
    parser.add_argument('--waypoint-timeout', type=float, default=30)
    parser.add_argument('--return-home', action='store_true',
                        help='Append start (0,0) as the last closed-loop waypoint')
    parser.add_argument('--test-emergency', action='store_true',
                        help='Send emergency stop on the ground, without arming')
    parser.add_argument('--height', type=float, default=0.3,
                        help='Hover height in metres (default 0.3)')
    parser.add_argument('--hover-seconds', type=float, default=3,
                        help='Hover duration after ascent (default 3)')
    parser.add_argument('--sensor', choices=('front', 'left', 'back', 'right'),
                        help='Highlight one lateral sensor during the hover')
    parser.add_argument('--flow-diagnostics', action='store_true',
                        help='Also log available optical-flow diagnostic channels')
    parser.add_argument('--forward', type=float, default=0,
                        help='After hover, advance this distance in metres at 0.1 m/s, then land')
    parser.add_argument('--square-side', type=float, default=0,
                        help='Fly one square: forward, left, back, right; side in metres')
    parser.add_argument('--rectangle', type=float, nargs=2, metavar=('FORWARD_M', 'LEFT_M'),
                        help='Fly one rectangle: forward, left, back, right; lengths in metres')
    parser.add_argument('--live-map', action='store_true',
                        help='Stream telemetry to the local live map via UDP')
    parser.add_argument('--map-port', type=int, default=8766)
    parser.add_argument('--decks-only', action='store_true',
                        help='Print deck detection and exit without starting logs')
    parser.add_argument('--seconds', type=float, default=5,
                        help='Telemetry-only recording duration (1-300 seconds)')
    parser.add_argument('--record', type=Path,
                        help='Create a new JSONL telemetry file; never overwrite')
    args = parser.parse_args()
    if args.navigate and not args.fly:
        parser.error('--navigate requires --fly')
    if args.return_home and (not args.navigate or not (args.forward or args.square_side or args.rectangle)):
        parser.error('--return-home requires --navigate and a route')
    if args.navigate and not .2 <= args.height <= .4:
        parser.error('--navigate mapping height must be between 0.2 and 0.4 m')
    if not math.isfinite(args.speed) or not 0 < args.speed <= .2:
        parser.error('--speed must be between 0 and 0.2 m/s')
    if not math.isfinite(args.waypoint_timeout) or not 5 <= args.waypoint_timeout <= 120:
        parser.error('--waypoint-timeout must be between 5 and 120 seconds')
    if sum((args.fly, args.decks_only, args.test_emergency)) > 1:
        parser.error('--fly, --decks-only and --test-emergency are mutually exclusive')
    if not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    if not math.isfinite(args.height) or args.height <= 0:
        parser.error('--height must be finite and positive')
    if not math.isfinite(args.hover_seconds) or args.hover_seconds <= 0:
        parser.error('--hover-seconds must be finite and positive')
    if not math.isfinite(args.forward) or args.forward < 0:
        parser.error('--forward must be finite and nonnegative')
    if args.forward and not args.fly:
        parser.error('--forward requires --fly')
    if not math.isfinite(args.square_side) or args.square_side < 0:
        parser.error('--square-side must be finite and nonnegative')
    if args.square_side and (not args.fly or args.forward):
        parser.error('--square-side requires --fly and cannot be combined with --forward')
    if args.rectangle:
        if not all(math.isfinite(v) and v > 0 for v in args.rectangle):
            parser.error('--rectangle lengths must be finite and positive')
        if not args.fly or args.forward or args.square_side:
            parser.error('--rectangle requires --fly; choose only one route')
    if not 1 <= args.map_port <= 65535:
        parser.error('--map-port must be between 1 and 65535')
    nav = None
    if args.navigate:
        from cf2_navigation import Navigation
        goals = [(args.forward, 0)] if args.forward else []
        if args.square_side or args.rectangle:
            a, b = args.rectangle or (args.square_side, args.square_side)
            goals = [(a, 0), (a, b), (0, b), (0, 0)]
        if args.return_home and goals[-1] != (0, 0):
            goals.append((0, 0))
        if any(abs(v) > 1.25 for goal in goals for v in goal):
            parser.error('Navigation goals must stay within 1.25 m of start')
        nav = Navigation(goals, speed=args.speed, height=args.height,
                         timeout=args.waypoint_timeout if goals else args.hover_seconds)
    session_id = uuid4().hex[:12]
    if args.fly:
        args.flow_diagnostics = True
        if args.record is None:
            args.record = Path('data/runs') / f'cf2_flight_{session_id}.jsonl'
    cflib.crtp.init_drivers()
    cache = Path(__file__).resolve().parents[1] / 'data' / 'runs' / 'cf2_cache'
    cache.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        recording = None
        record_lock = Lock()
        telemetry_lock = Lock()
        telemetry = deque(maxlen=2000)
        hovering = Event()
        phase = {'name': 'ground'}
        live_socket = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) if args.live_map else None
        if args.record:
            args.record.parent.mkdir(parents=True, exist_ok=True)
            recording = stack.enter_context(args.record.open('x'))
            print(f'Recording: {args.record}', flush=True)
            recording.write(json.dumps({'metadata': {'uri': args.uri,
                'pose_units': 'metres', 'yaw_units': 'degrees',
                'range_units': 'millimetres', 'fly': args.fly,
                'height': args.height, 'hover_seconds': args.hover_seconds,
                'selected_sensor': args.sensor, 'forward_metres': args.forward,
                'square_side_metres': args.square_side, 'rectangle_metres': args.rectangle,
                'navigation_enabled': args.navigate, 'speed': args.speed,
                'return_home': args.return_home,
                'session_id': session_id}}) + '\n')
        scf = stack.enter_context(SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(cache))))
        print('Connected; waiting for parameter download.', flush=True)
        scf.wait_for_params()
        if args.test_emergency:
            scf.cf.supervisor.send_emergency_stop()
            scf.cf.supervisor.send_arming_request(False)
            time.sleep(0.5)
            state = read_supervisor(scf.cf)
            print(f'Supervisor: locked={bool(state & 64)}, armed={bool(state & 2)}. '
                  'Restart the drone before another session.', flush=True)
            return
        for variable in ('stabilizer.estimator', 'stabilizer.controller'):
            group, name = variable.split('.')
            if name in scf.cf.param.toc.toc.get(group, {}):
                print(f'{variable}={scf.cf.param.get_value(variable)}', flush=True)
        if args.decks_only:
            state = read_supervisor(scf.cf)
            print(f'Supervisor: locked={bool(state & 64)}, armed={bool(state & 2)}, '
                  f'can_arm={bool(state & 1)}, can_fly={bool(state & 8)}', flush=True)
        deck_values = {}
        for name in sorted(scf.cf.param.toc.toc.get('deck', {})):
            value = scf.cf.param.get_value(f'deck.{name}')
            deck_values[name] = value
            print(f'deck.{name}={value}', flush=True)
        if args.decks_only:
            return
        missing = []
        for name in ('bcFlow2', 'bcMultiranger'):
            if int(deck_values.get(name, 0)) == 0:
                missing.append(name)
        if missing:
            raise RuntimeError(f'Required decks not detected: {", ".join(missing)}')

        events = []
        logs = []
        log_specs = [
            ('cf2_pose', ['stateEstimate.x', 'stateEstimate.y',
                          'stateEstimate.z', 'stateEstimate.yaw']),
            ('cf2_ranges', ['range.front', 'range.back', 'range.left',
                            'range.right', 'range.up', 'range.zrange']),
        ]
        if args.flow_diagnostics:
            candidates = ('motion.deltaX', 'motion.deltaY', 'motion.squal',
                          'motion.shutter', 'stateEstimate.vx',
                          'stateEstimate.vy', 'stateEstimate.vz')
            available = []
            for variable in candidates:
                group, name = variable.split('.')
                if name in scf.cf.log.toc.toc.get(group, {}):
                    available.append(variable)
                else:
                    print(f'Diagnostic channel unavailable: {variable}', flush=True)
            if available:
                log_specs.append(('cf2_flow', available))
        for name, variables in log_specs:
            event = Event()
            events.append(event)
            config = LogConfig(name=name, period_in_ms=100)
            for variable in variables:
                config.add_variable(variable)

            def received(timestamp, data, logconf, ready=event):
                sample = {'log': logconf.name, 'timestamp_ms': timestamp,
                                  'data': data, 'hovering': hovering.is_set(),
                                  'phase': phase['name'], 'session_id': session_id}
                if args.navigate:
                    with telemetry_lock:
                        telemetry.append((sample, time.monotonic()))
                row = json.dumps(sample)
                if live_socket:
                    try:
                        live_socket.sendto(row.encode(), ('127.0.0.1', args.map_port))
                    except OSError as error:
                        print(f'Live map transport error: {error}', flush=True)
                if args.sensor and args.fly:
                    if logconf.name == 'cf2_ranges' and hovering.is_set():
                        value = data.get(f'range.{args.sensor}')
                        reading = (f'{value} mm ({value / 1000:.3f} m)'
                                   if value is not None and 0 < value <= 4000
                                   else 'no usable return')
                        print(f'HOVER | {args.sensor}: {reading}', flush=True)
                else:
                    print(row, flush=True)
                if recording:
                    with record_lock:
                        recording.write(row + '\n')
                        recording.flush()
                ready.set()

            config.data_received_cb.add_callback(received)
            scf.cf.log.add_config(config)
            config.start()
            logs.append(config)
        try:
            if not all(event.wait(5) for event in events):
                raise RuntimeError('Missing pose/range telemetry; no takeoff')
            if args.fly:
                print(f'Takeoff to {args.height} m, hover for '
                      f'{args.hover_seconds} seconds, then land.', flush=True)
                scf.cf.supervisor.send_arming_request(True)
                time.sleep(1)
                try:
                    phase['name'] = 'takeoff'
                    with MotionCommander(scf, default_height=args.height) as motion:
                        hovering.set()
                        phase['name'] = 'hover'
                        try:
                            if args.navigate:
                                phase['name'] = 'navigation'
                                last_report = 0
                                print('CLOSED LOOP: mapping -> A* -> follower -> avoidance -> safety -> radio.', flush=True)
                                while True:
                                    now = time.monotonic()
                                    with telemetry_lock:
                                        samples = list(telemetry)
                                        telemetry.clear()
                                    for sample, received_at in samples:
                                        # Ignore estimator-reset and ascent data in navigation.
                                        if sample['hovering']:
                                            nav.update(sample, received_at)
                                    decision = nav.tick(now)
                                    if decision['land']:
                                        motion.stop()
                                        decision['state'] = 'LAND'
                                        row = json.dumps({'log': 'cf2_navigation', 'data': decision,
                                                          'session_id': session_id})
                                        if live_socket:
                                            live_socket.sendto(row.encode(), ('127.0.0.1', args.map_port))
                                        if recording:
                                            with record_lock:
                                                recording.write(row + '\n')
                                                recording.flush()
                                        snapshot = nav.mapper.snapshot(now)
                                        snapshot['navigation'] = decision
                                        snapshot['start'] = nav.origin
                                        snapshot['shared_frame_aligned'] = False
                                        map_file = args.record.with_name(args.record.stem + '_map.json')
                                        map_file.write_text(json.dumps(snapshot, allow_nan=False))
                                        print(f'Map snapshot: {map_file}', flush=True)
                                        print('NAV LAND: ' + decision['reason'], flush=True)
                                        break
                                    motion.start_linear_motion(*decision['velocity_body'])
                                    if now-last_report >= .5:
                                        last_report = now
                                        print(f"NAV: {decision['state']} | {decision['reason']} | "
                                              f"body velocity={decision['velocity_body']} | "
                                              f"waypoint={decision['waypoint_index']} | replans={decision['replans']}", flush=True)
                                        row = json.dumps({'log': 'cf2_navigation', 'data': decision,
                                                          'session_id': session_id})
                                        if live_socket:
                                            live_socket.sendto(row.encode(), ('127.0.0.1', args.map_port))
                                        if recording:
                                            with record_lock:
                                                recording.write(row + '\n')
                                    time.sleep(.05)
                            else:
                                print('OPEN LOOP: automatic obstacle avoidance is NOT enabled.', flush=True)
                            if not args.navigate:
                                print('HOVER START: sensor measurement window.', flush=True)
                                time.sleep(args.hover_seconds)
                            if args.forward and not args.navigate:
                                phase['name'] = 'forward'
                                print(f'FORWARD: {args.forward} m at 0.1 m/s.', flush=True)
                                motion.start_forward(velocity=0.1)
                                time.sleep(args.forward / 0.1)
                                motion.stop()
                                time.sleep(1)
                            if (args.square_side or args.rectangle) and not args.navigate:
                                forward_length, left_length = (args.rectangle or
                                                               (args.square_side, args.square_side))
                                for label, vx, vy, distance in [
                                    ('forward', 0.1, 0, forward_length),
                                    ('left', 0, 0.1, left_length),
                                    ('back', -0.1, 0, forward_length),
                                    ('right', 0, -0.1, left_length)]:
                                    phase['name'] = label
                                    print(f'ROUTE: {label}, {distance} m at 0.1 m/s.', flush=True)
                                    motion.start_linear_motion(vx, vy, 0)
                                    time.sleep(distance / 0.1)
                                    motion.stop()
                                    phase['name'] = 'hover'
                                    time.sleep(1)
                        finally:
                            hovering.clear()
                            phase['name'] = 'landing'
                            print('HOVER END: landing routine requested; '
                                  'actual descent is not confirmed.', flush=True)
                    print('Landing routine finished; verify physical touchdown.', flush=True)
                    phase['name'] = 'ground'
                    time.sleep(1)
                finally:
                    scf.cf.supervisor.send_arming_request(False)
            else:
                print('Telemetry only: motors remain unarmed.', flush=True)
                time.sleep(args.seconds)
        finally:
            for config in logs:
                config.stop()


if __name__ == '__main__':
    main()
