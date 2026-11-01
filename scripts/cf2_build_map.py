"""Build an exploratory local 2D map from one CF2 JSONL recording.

No-return/invalid ranges are skipped, never interpreted as clear space.
Pose yaw is degrees; ranges are millimetres. This is not SLAM.
"""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

ANGLES = {'front': 0, 'left': math.pi / 2,
          'back': math.pi, 'right': -math.pi / 2}


def ray_endpoint(pose, direction, distance):
    angle = math.radians(pose['stateEstimate.yaw']) + ANGLES[direction]
    return (pose['stateEstimate.x'] + distance * math.cos(angle),
            pose['stateEstimate.y'] + distance * math.sin(angle))


def build_map(rows, resolution=0.05, size=8.0, max_age_ms=50,
              min_height=None, max_height=None):
    poses = sorted((row['timestamp_ms'], row['data']) for row in rows
                   if row.get('log') == 'cf2_pose'
                   and all(math.isfinite(row['data'][key]) for key in
                           ('stateEstimate.x', 'stateEstimate.y',
                            'stateEstimate.z', 'stateEstimate.yaw'))
                   and (min_height is None or row['data']['stateEstimate.z'] >= min_height)
                   and (max_height is None or row['data']['stateEstimate.z'] <= max_height))
    if not poses:
        raise ValueError('No pose samples')
    stamps = [stamp for stamp, _ in poses]
    origin = (poses[0][1]['stateEstimate.x'] - size / 2,
              poses[0][1]['stateEstimate.y'] - size / 2)
    width = math.ceil(size / resolution)
    grid = [-1] * (width * width)
    stats = {'hits': 0, 'skipped_ranges': 0, 'unpaired_samples': 0,
             'out_of_bounds_rays': 0}

    def cell(x, y):
        cx = math.floor((x - origin[0]) / resolution)
        cy = math.floor((y - origin[1]) / resolution)
        if 0 <= cx < width and 0 <= cy < width:
            return cy * width + cx
        return None

    for row in rows:
        if row.get('log') != 'cf2_ranges':
            continue
        stamp = row['timestamp_ms']
        index = bisect_left(stamps, stamp)
        choices = [i for i in (index - 1, index) if 0 <= i < len(poses)]
        nearest = min(choices, key=lambda i: abs(stamps[i] - stamp))
        pose_stamp, pose = poses[nearest]
        if abs(pose_stamp - stamp) > max_age_ms:
            stats['unpaired_samples'] += 1
            continue
        if not all(math.isfinite(pose[key]) for key in
                   ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.yaw')):
            stats['unpaired_samples'] += 1
            continue
        start = (pose['stateEstimate.x'], pose['stateEstimate.y'])
        for direction in ANGLES:
            raw = row['data'].get(f'range.{direction}')
            if raw is None or not math.isfinite(raw) or not 50 <= raw <= 4000:
                stats['skipped_ranges'] += 1
                continue
            distance = raw / 1000
            end = ray_endpoint(pose, direction, distance)
            target = cell(*end)
            if target is None or cell(*start) is None:
                stats['out_of_bounds_rays'] += 1
                continue
            steps = max(1, math.ceil(distance / (resolution / 2)))
            for step in range(steps):
                fraction = step / steps
                pos = cell(start[0] + (end[0] - start[0]) * fraction,
                           start[1] + (end[1] - start[1]) * fraction)
                if pos is not None and pos != target and grid[pos] != 100:
                    grid[pos] = 0
            grid[target] = 100
            stats['hits'] += 1
    return grid, width, origin, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--min-height', type=float, help='Minimum pose height in metres')
    parser.add_argument('--max-height', type=float, help='Maximum pose height in metres')
    parser.add_argument('--hover-only', action='store_true',
                        help='Use only the airborne measurement window (includes forward motion)')
    args = parser.parse_args()
    if any(value is not None and not math.isfinite(value)
           for value in (args.min_height, args.max_height)):
        parser.error('Height limits must be finite')
    if (args.min_height is not None and args.max_height is not None
            and args.min_height > args.max_height):
        parser.error('--min-height must not exceed --max-height')
    rows = [json.loads(line) for line in args.recording.read_text().splitlines()
            if line.strip()]
    if args.hover_only:
        rows = [row for row in rows if row.get('hovering') is True]
    grid, width, origin, stats = build_map(rows, min_height=args.min_height,
                                          max_height=args.max_height)
    args.output.mkdir(parents=True, exist_ok=False)
    colours = {-1: 205, 0: 254, 100: 0}
    pixels = bytes(colours[grid[y * width + x]]
                   for y in reversed(range(width)) for x in range(width))
    (args.output / 'map.pgm').write_bytes(f'P5\n{width} {width}\n255\n'.encode() + pixels)
    (args.output / 'map.json').write_text(json.dumps({
        'frame': 'cf2_local', 'resolution': 0.05, 'origin': origin,
        'width': width, 'height': width, 'data': grid, 'stats': stats}, indent=2))
    (args.output / 'map.yaml').write_text(
        f'image: map.pgm\nresolution: 0.05\norigin: [{origin[0]}, {origin[1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(json.dumps(stats, indent=2))
    print(f'Map saved to {args.output}. Gray=unknown, white=free, black=occupied.')


if __name__ == '__main__':
    main()
