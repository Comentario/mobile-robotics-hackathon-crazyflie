"""Offline CF2 map route preview. Publishes visualization only, never flight commands.

Frames are aligned from measured A/B initial poses, not by renaming a frame.
Only a measured height layer is used; unknown space is blocked.
"""
import argparse
import heapq
import json
import math
from pathlib import Path
import time

import yaml


def finite(values):
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Coordinates must be finite')


def alignment(snapshot, reference_a, reference_b):
    if reference_a['session_id'] != snapshot['session_id']:
        raise ValueError('A reference belongs to a different map session')
    a = reference_a['initial_pose']
    pose = reference_b['pose']
    b, q = pose['position'], pose['orientation']
    if reference_b['header']['frame_id'] != 'world':
        raise ValueError('B reference must be /cf231/pose in world (aligned with map)')
    finite([a['x'], a['y'], a['yaw_deg'], b['x'], b['y'], *q.values()])
    norm = math.sqrt(sum(q[k] ** 2 for k in ('x', 'y', 'z', 'w')))
    if norm < 1e-6:
        raise ValueError('Invalid B orientation')
    x, y, z, w = (q[k] / norm for k in ('x', 'y', 'z', 'w'))
    yaw_b = math.atan2(2 * (w*z+x*y), 1-2*(y*y+z*z))
    angle = yaw_b - math.radians(a['yaw_deg'])
    c, s = math.cos(angle), math.sin(angle)
    return [b['x']-c*a['x']+s*a['y'], b['y']-s*a['x']-c*a['y'], angle]


def transform(point, tf, inverse=False):
    tx, ty, angle = tf
    c, s = math.cos(angle), math.sin(angle)
    x, y = point[:2]
    if inverse:
        x, y = x-tx, y-ty
        return [c*x+s*y, -s*x+c*y, point[2]]
    return [c*x-s*y+tx, s*x+c*y+ty, point[2]]


def select_layer(snapshot, height):
    for layer in snapshot.get('layers', []):
        measured_height = float(layer['height_m'])
        tolerance = float(layer.get('tolerance_m', 0.045))
        if not math.isfinite(tolerance) or not 0 < tolerance <= 0.05:
            raise ValueError('Invalid layer height tolerance')
        if abs(measured_height-height) <= tolerance:
            return layer['data'], measured_height
    raise ValueError(f'No measured map layer at {height:.2f} m; cannot reuse another height')


def inflate(data, width, height, resolution, clearance):
    """Conservative cell-footprint dilation, including unknown and map boundaries.

    clearance = vehicle horizontal radius + extra margin/uncertainty, in metres.
    sqrt(2)*resolution accounts for both source and destination cell footprints.
    """
    reach = clearance + math.sqrt(2)*resolution
    n = math.ceil(reach/resolution)
    offsets = [(dx, dy) for dy in range(-n, n+1) for dx in range(-n, n+1)
               if math.hypot(dx, dy)*resolution <= reach]
    blocked = [v != 0 for v in data]
    for y in range(height):
        for x in range(width):
            if not blocked[y*width+x] and any(
                not (0 <= x+dx < width and 0 <= y+dy < height)
                for dx, dy in offsets
            ):
                blocked[y*width+x] = True
    # Dilate from the original map only; otherwise dilation cascades.
    for i, value in enumerate(data):
        if value == 0:
            continue
        x, y = i % width, i // width
        for dx, dy in offsets:
            nx, ny = x+dx, y+dy
            if 0 <= nx < width and 0 <= ny < height:
                blocked[ny*width+nx] = True
    return blocked


def astar(blocked, width, height, start, goal, timeout=5.0):
    if blocked[start[1]*width+start[0]] or blocked[goal[1]*width+goal[0]]:
        raise ValueError('Start or waypoint is occupied, unknown, or inside the clearance margin')
    deadline = time.monotonic()+timeout
    costs, parents = {start: 0.0}, {}
    queue = [(math.dist(start, goal), 0.0, start)]
    while queue:
        if time.monotonic() > deadline:
            raise ValueError('Planning timeout; no complete route produced')
        _, cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            return path[::-1]
        x, y = current
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = x+dx, y+dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if blocked[ny*width+nx]:
                continue
            if dx and dy and (blocked[y*width+nx] or blocked[ny*width+x]):
                continue
            nxt, candidate = (nx, ny), cost+math.hypot(dx, dy)
            if candidate < costs.get(nxt, math.inf):
                costs[nxt], parents[nxt] = candidate, current
                heapq.heappush(queue, (candidate+math.dist(nxt, goal), candidate, nxt))
    raise ValueError('No route through measured free space with the requested clearance')


def plan(snapshot, mission, start, tf, robot_radius, margin, allow_simulation=False,
         geofence=(-2., -2., 2., 2.)):
    simulation = snapshot.get('simulation') or snapshot.get('navigation', {}).get('simulation')
    if simulation and not allow_simulation:
        raise ValueError('Simulation map: use --allow-simulation only for software preview')
    if mission['frame_id'] != 'map':
        raise ValueError('Mission must use Lighthouse-aligned map coordinates')
    targets = mission['waypoints']
    if not targets or any(len(p) != 3 for p in [start]+targets):
        raise ValueError('Supply a start and at least one XYZ waypoint')
    for p in [start]+targets:
        finite(p)
    z = float(targets[0][2])
    if any(abs(float(p[2])-z) > 1e-6 for p in [start]+targets):
        raise ValueError('All points must share one navigation height, including airborne start')
    if not (math.isfinite(robot_radius) and robot_radius > 0 and
            math.isfinite(margin) and margin >= 0):
        raise ValueError('Supply a positive measured vehicle radius and nonnegative extra margin')
    data, layer_z = select_layer(snapshot, z)
    width, height, res = snapshot['width'], snapshot['height'], snapshot['resolution']
    origin = snapshot['origin']
    if not (isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0
            and math.isfinite(res) and res > 0 and len(data) == width*height):
        raise ValueError('Invalid map dimensions or resolution')
    finite([*origin, *tf])
    if any(v not in (-1, 0, 100) for v in data):
        raise ValueError('Expected measured occupancy values -1, 0, 100')
    blocked = inflate(data, width, height, res, robot_radius+margin)
    finite(geofence)
    xmin, ymin, xmax, ymax = geofence
    if xmin >= xmax or ymin >= ymax:
        raise ValueError('Invalid XY geofence')
    fence_margin = robot_radius+margin+res/math.sqrt(2)
    for cy in range(height):
        for cx in range(width):
            world = transform([origin[0]+(cx+.5)*res, origin[1]+(cy+.5)*res, z], tf)
            if not (xmin+fence_margin <= world[0] <= xmax-fence_margin and
                    ymin+fence_margin <= world[1] <= ymax-fence_margin):
                blocked[cy*width+cx] = True

    def cell(point):
        local = transform(point, tf, inverse=True)
        x, y = math.floor((local[0]-origin[0])/res), math.floor((local[1]-origin[1])/res)
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError('Start or waypoint lies outside the map')
        return x, y

    previous, route, legs = start, [list(start)], []
    for index, target in enumerate(targets):
        cells = astar(blocked, width, height, cell(previous), cell(target))
        points = [transform([origin[0]+(x+.5)*res, origin[1]+(y+.5)*res, z], tf)
                  for x, y in cells]
        # Exact anchors are inside free cells; centre connectors remain in those cells.
        segment = [list(previous)]+points+[list(target)]
        route.extend(segment[1:])
        legs.append({'waypoint_index': index, 'points': segment})
        previous = target
    return dict(frame_id='map', session_id=snapshot['session_id'], simulation=bool(simulation),
                layer_height_m=layer_z, navigation_height_m=z, transform=tf,
                robot_radius_m=robot_radius, extra_margin_m=margin,
                clearance_m=robot_radius+margin, unknown_policy='block',
                geofence_xy=list(geofence),
                initial=[list(start)]+targets, modified=route, legs=legs,
                grid=dict(width=width, height=height, resolution=res, origin=origin,
                          raw=data, inflated=[100 if b else 0 for b in blocked]),
                purpose='Visualization preview only; not connected to flight control')


def publish(result):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node('shared_map_route_preview')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {key: node.create_publisher(RosPath, topic, qos) for key, topic in (
        ('initial', '/hk/path/initial'), ('modified', '/hk/path/modified'))}
    if 'proposed' in result:
        pubs['proposed'] = node.create_publisher(RosPath, '/hk/path/proposed', qos)
    from std_msgs.msg import String
    status_pub = node.create_publisher(String, '/hk/shared_map/preview_status', qos)
    maps = {key: node.create_publisher(OccupancyGrid, topic, qos) for key, topic in (
        ('raw', '/hk/shared_map/raw'), ('inflated', '/hk/shared_map/inflated'))}

    def tick():
        stamp = node.get_clock().now().to_msg()
        status = String()
        status.data = result.get('status', 'Preview only; not a flight command')
        status_pub.publish(status)
        for key, pub in pubs.items():
            msg = RosPath()
            msg.header.frame_id, msg.header.stamp = 'map', stamp
            for x, y, z in result[key]:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
            pub.publish(msg)
        grid = result['grid']
        origin = transform([*grid['origin'], 0.0], result['transform'])
        angle = result['transform'][2]
        for key, pub in maps.items():
            msg = OccupancyGrid()
            msg.header.frame_id, msg.header.stamp = 'map', stamp
            msg.info.width, msg.info.height = grid['width'], grid['height']
            msg.info.resolution = float(grid['resolution'])
            msg.info.origin.position.x, msg.info.origin.position.y = origin[:2]
            msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(angle/2), math.cos(angle/2)
            msg.data = grid[key]
            pub.publish(msg)

    node.create_timer(1.0, tick)
    tick()
    node.get_logger().info('Preview only: /hk/path/initial, /hk/path/modified and /hk/shared_map/{raw,inflated}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mission', type=Path, required=True)
    parser.add_argument('--reference-a', type=Path, required=True)
    parser.add_argument('--reference-b', type=Path, required=True)
    parser.add_argument('--start', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--robot-radius', type=float, required=True)
    parser.add_argument('--margin', type=float, default=0.15,
                        help='Extra clearance for map/alignment/tracking uncertainty, metres')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geofence', type=float, nargs=4, default=[-2., -2., 2., 2.],
                        metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'))
    parser.add_argument('--publish', action='store_true')
    parser.add_argument('--allow-simulation', action='store_true')
    args = parser.parse_args()
    try:
        snapshot = json.loads(args.map.read_text())
        mission = yaml.safe_load(args.mission.read_text())
        a = yaml.safe_load(args.reference_a.read_text())
        docs = yaml.safe_load_all(args.reference_b.read_text())
        b = next(doc for doc in docs if doc is not None)
        result = plan(snapshot, mission, args.start, alignment(snapshot, a, b),
                      args.robot_radius, args.margin, args.allow_simulation, args.geofence)
        args.output.write_text(json.dumps(result, indent=2))
    except (ValueError, KeyError, TypeError, OSError, StopIteration, yaml.YAMLError) as error:
        parser.exit(2, f'No complete route generated: {error}\n')
    print(f'Preview saved: {args.output}; {len(result["legs"])} waypoints; '
          f'clearance {result["clearance_m"]:.2f} m. Not a flight command.')
    if args.publish:
        publish(result)


if __name__ == '__main__':
    main()
