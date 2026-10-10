"""Incremental CF2 map, browser display and optional ROS 2 publication.

Receives JSON telemetry over local UDP; never connects to or commands a drone.
"""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
from threading import Event, Lock, Thread
import time

from cf2_build_map import ANGLES, ray_endpoint


class LiveMap:
    def __init__(self, resolution=0.05, width=160, min_height=0.2, max_height=0.4,
                 decay_seconds=120):
        self.resolution, self.width = resolution, width
        self.min_height, self.max_height = min_height, max_height
        self.decay_seconds = decay_seconds
        self.lock = Lock()
        self.reset('waiting')

    def reset(self, session):
        self.session = session
        self.origin = None
        self.scores = [0] * self.width ** 2
        self.updated = [None] * self.width ** 2
        self.poses = deque(maxlen=20)
        self.trail = deque(maxlen=2000)
        self.pose, self.ranges = None, {}
        self.start, self.navigation = None, None
        self.last_received = None
        self.last_pose_at = self.last_ranges_at = None
        self.hit_history = {}
        self.recent_hits = deque(maxlen=400)
        self.stats = {'hits': 0, 'invalid_ranges': 0, 'unpaired': 0,
                      'outside_map': 0, 'mapped_samples': 0}

    def cell(self, x, y):
        cx = math.floor((x - self.origin[0]) / self.resolution)
        cy = math.floor((y - self.origin[1]) / self.resolution)
        if 0 <= cx < self.width and 0 <= cy < self.width:
            return cy * self.width + cx
        return None

    def update(self, row, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            session = row.get('session_id', 'default')
            if session != self.session:
                self.reset(session)
            self.last_received = now
            name, data = row.get('log'), row.get('data', {})
            if name == 'cf2_navigation':
                self.navigation = dict(data)
                return
            if name == 'cf2_pose':
                keys = ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
                        'stateEstimate.yaw')
                if not all(key in data and math.isfinite(data[key]) for key in keys):
                    return
                self.pose = dict(data)
                self.last_pose_at = now
                self.poses.append((row['timestamp_ms'], dict(data), row.get('hovering', False)))
                if row.get('hovering') and self.min_height <= data['stateEstimate.z'] <= self.max_height:
                    if self.origin is None:
                        half = self.width * self.resolution / 2
                        self.origin = (data['stateEstimate.x'] - half,
                                       data['stateEstimate.y'] - half)
                        self.start = (data['stateEstimate.x'], data['stateEstimate.y'])
                    self.trail.append((data['stateEstimate.x'], data['stateEstimate.y']))
                return
            if name != 'cf2_ranges':
                return
            self.ranges = dict(data)
            self.last_ranges_at = now
            if not row.get('hovering') or not self.poses or self.origin is None:
                return
            stamp, pose, airborne = min(self.poses, key=lambda item: abs(item[0] - row['timestamp_ms']))
            if abs(stamp - row['timestamp_ms']) > 50 or not airborne:
                self.stats['unpaired'] += 1
                return
            if not self.min_height <= pose['stateEstimate.z'] <= self.max_height:
                return
            self.stats['mapped_samples'] += 1
            start = (pose['stateEstimate.x'], pose['stateEstimate.y'])
            for direction in ANGLES:
                raw = data.get(f'range.{direction}')
                if raw is None or not math.isfinite(raw) or not 50 <= raw <= 4000:
                    self.stats['invalid_ranges'] += 1
                    continue
                end = ray_endpoint(pose, direction, raw / 1000)
                target = self.cell(*end)
                if target is None or self.cell(*start) is None:
                    self.stats['outside_map'] += 1
                    continue
                steps = max(1, math.ceil(raw / 1000 / (self.resolution / 2)))
                free = set()
                for step in range(steps):
                    fraction = step / steps
                    cell = self.cell(start[0] + (end[0] - start[0]) * fraction,
                                     start[1] + (end[1] - start[1]) * fraction)
                    if cell is not None and cell != target:
                        free.add(cell)
                for cell, delta in [(cell, -1) for cell in free] + [(target, 3)]:
                    previous = self.updated[cell]
                    if previous is None or now - previous > self.decay_seconds:
                        self.scores[cell] = 0
                    self.scores[cell] = max(-4, min(6, self.scores[cell] + delta))
                    self.updated[cell] = now
                    if cell in free and self.scores[cell] <= 0:
                        self.hit_history.pop(cell, None)
                history = self.hit_history.setdefault(target, deque(maxlen=32))
                if history and now-history[-1] > 1:
                    history.clear()
                history.append(now)
                self.recent_hits.append((now, *end))
                self.stats['hits'] += 1

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            grid = [-1 if stamp is None or now - stamp > self.decay_seconds
                    else (100 if self.scores[i] > 0 else 0)
                    for i, stamp in enumerate(self.updated)]
            # Presentation layer only: repeated observations, never synthetic walls.
            persistent = []
            for i, value in enumerate(grid):
                history = self.hit_history.get(i)
                confirmed = bool(history and len(history) >= 4 and
                                 history[-1]-history[0] >= .6 and now-history[-1] <= 5)
                persistent.append(100 if value == 100 and confirmed else (-1 if value == 100 else value))
            while self.recent_hits and now-self.recent_hits[0][0] > 1.2:
                self.recent_hits.popleft()
            return {'session_id': self.session, 'frame': f'cf2_local_{self.session}',
                    'resolution': self.resolution, 'width': self.width, 'height': self.width,
                    'origin': self.origin, 'data': grid, 'pose': self.pose,
                    'presentation_data': persistent,
                    'recent_hits': [(x, y) for _, x, y in self.recent_hits],
                    'ranges_mm': dict(self.ranges), 'trail': list(self.trail),
                    'start': self.start, 'navigation': self.navigation,
                    'stats': dict(self.stats), 'age_seconds': None if
                    self.last_pose_at is None or self.last_ranges_at is None else
                    max(now-self.last_pose_at, now-self.last_ranges_at),
                    'decay_seconds': self.decay_seconds}


class RosPublisher:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import Range
        from hackathon_interfaces.msg import MultiRange
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.rclpy, self.Grid, self.Odom, self.Range = rclpy, OccupancyGrid, Odometry, Range
        rclpy.init()
        self.node = rclpy.create_node('cf2_live_map')
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/cf2/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pose_pub = self.node.create_publisher(Odometry, '/cf2/odom', 10)
        self.MultiRange = MultiRange
        self.ranges_pub = self.node.create_publisher(MultiRange, '/cf2/ranges', 10)
        self.range_pubs = {name: self.node.create_publisher(Range, f'/cf2/range/{name}', 10)
                           for name in ANGLES}
        from cf2_rviz import RvizPublisher
        self.rviz = RvizPublisher(self.node)

    def publish(self, state):
        self.rviz.publish(state)
        if state['origin'] is None or state['age_seconds'] is None or state['age_seconds'] > 1:
            return
        stamp = self.node.get_clock().now().to_msg()
        grid = self.Grid()
        grid.header.stamp, grid.header.frame_id = stamp, state['frame']
        grid.info.resolution = state['resolution']
        grid.info.width, grid.info.height = state['width'], state['height']
        grid.info.origin.position.x, grid.info.origin.position.y = state['origin']
        grid.info.origin.orientation.w = 1.0
        grid.data = state['data']
        self.map_pub.publish(grid)
        pose = state['pose']
        odom = self.Odom()
        odom.header = grid.header
        odom.child_frame_id = 'cf2_base_link'
        for axis in 'xyz':
            setattr(odom.pose.pose.position, axis, float(pose[f'stateEstimate.{axis}']))
        yaw = math.radians(pose['stateEstimate.yaw'])
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        # Uncertainty is not measured: conservative placeholders, not EKF covariance.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.25
        odom.pose.covariance[14] = 0.04
        odom.pose.covariance[35] = 0.1
        self.pose_pub.publish(odom)
        ranges = self.MultiRange()
        ranges.header.stamp = stamp
        ranges.header.frame_id = 'cf2_base_link'
        ranges.range_min, ranges.range_max = 0.05, 4.0
        for direction in ('front', 'back', 'left', 'right', 'up', 'down'):
            raw = state['ranges_mm'].get('range.zrange' if direction == 'down' else f'range.{direction}')
            # Unknown returns remain unknown; never turn sentinel 32766 into free space.
            value = raw / 1000 if raw is not None and 50 <= raw <= 4000 else float('nan')
            setattr(ranges, direction, value)
        self.ranges_pub.publish(ranges)
        for direction, publisher in self.range_pubs.items():
            raw = state['ranges_mm'].get(f'range.{direction}')
            if raw is None or not 50 <= raw <= 4000:
                continue
            msg = self.Range()
            msg.header.stamp = stamp
            msg.header.frame_id = f'cf2_{direction}_range'
            msg.radiation_type = self.Range.INFRARED
            msg.field_of_view, msg.min_range, msg.max_range = math.radians(27), 0.05, 4.0
            msg.range = raw / 1000
            publisher.publish(msg)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--udp-port', type=int, default=8766)
    parser.add_argument('--ros', action='store_true', help='Publish /cf2/map and /cf2/odom')
    parser.add_argument('--decay-seconds', type=float, default=120,
                        help='Observation lifetime in seconds (default 120)')
    parser.add_argument('--replay', type=Path, help='Replay a recording for software verification; no flight')
    args = parser.parse_args()
    if not math.isfinite(args.decay_seconds) or args.decay_seconds <= 0:
        parser.error('--decay-seconds must be finite and positive')
    mapper, done = LiveMap(decay_seconds=args.decay_seconds), Event()
    page = Path(__file__).with_name('cf2_live_map.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/state':
                body, mime = json.dumps(mapper.snapshot(), allow_nan=False).encode(), 'application/json'
            elif self.path == '/export':
                snapshot = mapper.snapshot()
                snapshot['coordinate_reference'] = 'Session EKF frame; start stores the local XY reference'
                snapshot['shared_frame_aligned'] = False
                body, mime = json.dumps(snapshot, allow_nan=False).encode(), 'application/json'
            elif self.path == '/':
                body, mime = page, 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            if self.path == '/export':
                self.send_header('Content-Disposition', 'attachment; filename="cf2-map.json"')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', args.udp_port))
    udp.settimeout(0.2)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    ros = RosPublisher() if args.ros else None

    def receive():
        while not done.is_set():
            try:
                packet, _ = udp.recvfrom(65535)
                mapper.update(json.loads(packet))
            except socket.timeout:
                pass
            except OSError:
                if done.is_set():
                    return
                raise
            except (ValueError, KeyError, TypeError) as error:
                print(f'Skipped malformed telemetry: {error}', flush=True)

    def replay():
        rows = [json.loads(line) for line in args.replay.read_text().splitlines()]
        previous = None
        for row in rows:
            if 'timestamp_ms' not in row:
                continue
            stamp = row['timestamp_ms']
            if previous is not None and done.wait(max(0, min(1, (stamp - previous) / 1000))):
                return
            row['session_id'] = args.replay.stem
            mapper.update(row)
            previous = stamp
        print('Replay finished; page remains available. This is recorded data.', flush=True)

    Thread(target=server.serve_forever, daemon=True).start()
    Thread(target=replay if args.replay else receive, daemon=True).start()
    print(f'Live map: http://localhost:{args.port} | UDP 127.0.0.1:{args.udp_port}', flush=True)
    try:
        while not done.wait(0.1 if ros else 0.2):
            if ros:
                ros.publish(mapper.snapshot())
    except KeyboardInterrupt:
        pass
    finally:
        done.set()
        server.shutdown()
        server.server_close()
        udp.close()
        if ros:
            ros.close()


if __name__ == '__main__':
    main()
