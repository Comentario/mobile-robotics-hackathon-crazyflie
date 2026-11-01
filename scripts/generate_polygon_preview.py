"""Generate a CF2-format synthetic polygon map and route previews, without radio.

The requested waypoints are preserved. A separately labelled proposal moves
blocked waypoints to the nearest available cell for visualization only.
"""
import argparse
import json
import math
from pathlib import Path

import yaml

from plan_shared_map import inflate, plan, publish

POLYGON = [(2.24, -1.03), (2.16, 1.03), (-1.75, .85), (-1.60, -.60)]
OBSTACLE = (-.115, .615, .385, .815)  # xmin, ymin, xmax, ymax
WAYPOINTS = [[1.43, -.6, .8], [1.57, .8, .8], [-1.3, .63, .8],
             [-1.5, 0., .8], [.03, -.09, .8]]
START = [.03, -.09, .8]  # Synthetic start, not a measurement of B.
GEOFENCE = (-2., -2., 2., 2.)  # Match B's existing configured XY envelope.


def inside_polygon(x, y):
    return all((bx-ax)*(y-ay)-(by-ay)*(x-ax) >= -1e-12
               for (ax, ay), (bx, by) in zip(POLYGON, POLYGON[1:]+POLYGON[:1]))


def make_map(resolution=.05, width=160):
    origin = [-4., -4.]
    data = []
    xmin, ymin, xmax, ymax = OBSTACLE
    for cy in range(width):
        for cx in range(width):
            x, y = origin[0]+cx*resolution, origin[1]+cy*resolution
            corners = ((x, y), (x+resolution, y), (x, y+resolution),
                       (x+resolution, y+resolution))
            intersects = x <= xmax and x+resolution >= xmin and y <= ymax and y+resolution >= ymin
            data.append(0 if all(inside_polygon(*p) for p in corners) and not intersects else 100)
    stats = dict(free_cells=data.count(0), occupied_cells=data.count(100), source='synthetic geometry')
    layer = dict(index=0, height_m=.8, tolerance_m=.045, data=data,
                 presentation_data=list(data), trail=[], stats=stats)
    return dict(session_id='synthetic_lighthouse_polygon', frame='map', resolution=resolution,
                width=width, height=width, origin=origin, data=data,
                presentation_data=list(data), layers=[layer], active_layer=0,
                survey_heights=[.8], start=START[:2], pose=None, ranges_mm={},
                trail=[], recent_hits=[], recent_observations=[], navigation=dict(simulation=True),
                stats=stats, age_seconds=None, decay_seconds=None, simulation=True,
                shared_frame_aligned=True, map_validated_for_drone1=False,
                composite_is_visualization_only=True,
                height_reference='Synthetic Lighthouse-aligned Z, not measured',
                polygon=[list(p) for p in POLYGON], synthetic_obstacle=list(OBSTACLE),
                provenance='Synthetic geometry in CF2 export format; not a physical survey')


def generate(snapshot, radius=.10, margin=.02):
    mission = dict(frame_id='map', waypoints=WAYPOINTS)
    resolution, width, height, origin = (snapshot[k] for k in ('resolution', 'width', 'height', 'origin'))
    data = snapshot['layers'][0]['data']
    blocked = inflate(data, width, height, resolution, radius+margin)
    # Apply the same geofence/cell-footprint margin as the planner.
    boundary = radius+margin+resolution/math.sqrt(2)
    cells = []
    for cy in range(height):
        for cx in range(width):
            x, y = origin[0]+(cx+.5)*resolution, origin[1]+(cy+.5)*resolution
            index = cy*width+cx
            if not (GEOFENCE[0]+boundary <= x <= GEOFENCE[2]-boundary and
                    GEOFENCE[1]+boundary <= y <= GEOFENCE[3]-boundary):
                blocked[index] = True
            if not blocked[index]:
                cells.append([x, y, .8])
    if not cells:
        raise ValueError('No free cells remain with the requested clearance')
    proposed, changes = [], []
    for index, point in enumerate(WAYPOINTS):
        cx, cy = math.floor((point[0]-origin[0])/resolution), math.floor((point[1]-origin[1])/resolution)
        valid = 0 <= cx < width and 0 <= cy < height and not blocked[cy*width+cx]
        nearest = point if valid else min(cells, key=lambda p: math.dist(p[:2], point[:2]))
        proposed.append(list(nearest))
        if not valid:
            changes.append(dict(waypoint_number=index+1, requested=point, proposed=nearest,
                                displacement_m=math.dist(point[:2], nearest[:2]),
                                reason='Blocked by boundary, obstacle, or clearance'))
    candidate = plan(snapshot, dict(frame_id='map', waypoints=proposed), START,
                     [0., 0., 0.], radius, margin, allow_simulation=True, geofence=GEOFENCE)
    try:
        requested = plan(snapshot, mission, START, [0., 0., 0.], radius, margin,
                         allow_simulation=True, geofence=GEOFENCE)
        requested['status'] = 'SYNTHETIC PREVIEW: original mission has a route; no flight commands'
    except ValueError as error:
        requested = dict(candidate, initial=[START]+WAYPOINTS, modified=[], legs=[],
                         status=f'SYNTHETIC PREVIEW: ORIGINAL MISSION BLOCKED ({error}); '
                                'proposed path changes waypoints and is not approved for flight')
    requested.update(proposed=candidate['modified'], requested_waypoints=WAYPOINTS,
                     proposed_waypoints=proposed, waypoint_changes=changes)
    return requested, candidate


def plot(snapshot, result, target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    from matplotlib.colors import ListedColormap
    import numpy as np

    fig, ax = plt.subplots(figsize=(10, 5))
    grid = result['grid']
    x0, y0 = grid['origin']
    ax.imshow(np.asarray(grid['inflated']).reshape(grid['height'], grid['width']),
              extent=(x0, x0+grid['width']*grid['resolution'], y0, y0+grid['height']*grid['resolution']),
              origin='lower', cmap=ListedColormap(['#ffffff', '#eeeeee']), vmin=0, vmax=100)
    ax.add_patch(Polygon(POLYGON, closed=True, fill=False, edgecolor='black', linewidth=2))
    x, y, xmax, ymax = OBSTACLE
    ax.add_patch(Rectangle((x, y), xmax-x, ymax-y, facecolor='#333333', label='Synthetic obstacle'))
    route_label = ('Proposal with corrected points' if result['waypoint_changes']
                   else 'Computed route: same waypoints')
    for key, color, style, label in [('initial', '#c93232', '--', 'Requested direct route'),
                                    ('proposed', '#167b46', '-', route_label)]:
        points = result[key]
        ax.plot([p[0] for p in points], [p[1] for p in points], style, color=color, label=label)
    for index, (original, proposal) in enumerate(zip(WAYPOINTS, result['proposed_waypoints']), 1):
        ax.scatter(*original[:2], color='#c93232', marker='x', s=60)
        ax.annotate(str(index), original[:2], xytext=(4, 5), textcoords='offset points')
        if original != proposal:
            ax.scatter(*proposal[:2], color='#167b46', s=25)
            ax.plot([original[0], proposal[0]], [original[1], proposal[1]], ':', color='#555555')
    ax.set(xlim=(-1.95, 2.4), ylim=(-.9, 1.3), xlabel='X Lighthouse (m)', ylabel='Y Lighthouse (m)',
           title=f'Synthetic map — Z=0.8 m — clearance={result["clearance_m"]:.2f} m\n'
                 'Original points kept; no flight commands')
    ax.set_aspect('equal')
    ax.legend(loc='lower center', fontsize=8)
    fig.tight_layout()
    fig.savefig(target, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('data/examples/cooperative_polygon'))
    parser.add_argument('--robot-radius', type=float, default=.10,
                        help='Synthetic illustrative radius, not a measurement of the real drone')
    parser.add_argument('--margin', type=float, default=.02)
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--publish', action='store_true')
    args = parser.parse_args()
    if not (math.isfinite(args.robot_radius) and args.robot_radius > 0 and
            math.isfinite(args.margin) and args.margin >= 0):
        parser.error('Invalid radius or margin')
    snapshot = make_map()
    result, candidate = generate(snapshot, args.robot_radius, args.margin)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, value in [('cf2_synthetic_polygon_map.json', snapshot),
                        ('route_preview.json', result), ('route_proposal.json', candidate)]:
        (args.output_dir/name).write_text(json.dumps(value, indent=2))
    for name, points in [('requested_waypoints.yaml', WAYPOINTS),
                         ('proposed_waypoints.yaml', result['proposed_waypoints'])]:
        (args.output_dir/name).write_text(yaml.safe_dump(dict(frame_id='map', waypoints=points)))
    print(result['status'])
    print('Clearance:', result['clearance_m'], 'm (illustrative radius + extra margin)')
    for change in result['waypoint_changes']:
        print(f'Waypoint {change["waypoint_number"]}: {change["requested"]} → '
              f'{change["proposed"]} (proposal only)')
    print('Files:', args.output_dir.resolve())
    if args.plot:
        plot(snapshot, result, args.output_dir/'preview.png')
    if args.publish:
        publish(result)


if __name__ == '__main__':
    main()
