# CF2: one-height live mapping of a 5 × 3 m rectangle

Work stays on `integration/cf2-navigation-mapping`. CF2 uses Flow/EKF and the
four lateral Multiranger sensors; no Lighthouse is required. The flight
script owns the radio. RViz receives telemetry only.

## Run the current profile

Mark the rectangle and place CF2 in clear space at its centre. X/Y follow the
session EKF axes; X spans 5 m and Y spans 3 m. The first navigation pose defines
the home reference. This is a configured boundary, not a detected wall.
Keep the fixed obstacle visible at the 0.30 m sensor plane. Mesh gaps and narrow
chair legs can produce no useful return; start with a broad fixed panel/box.
Close cfclient and previous mapper/RViz processes.

Terminal 1:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
bash scripts/cf2_view_live.sh
```

Terminal 2:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --navigate --live-map \
  --height 0.3 --survey-heights 0.3 \
  --speed 0.15 --warning-distance 0.5 \
  --mapping-radius 0.8 --scan-yaw-rate 12 \
  --coverage-area 5 3 --coverage-spacing 0.5 \
  --survey-seconds 180 --waypoint-timeout 30
```

This moves and rotates simultaneously, while mapping incoming measurements
continuously. There are no stationary scan pauses. A 12 degree/s rotation
makes one turn in 30 s; slower translation reduces the gaps between successive
beam orientations. Rotation pauses during critical retreat, localization or
telemetry problems, height changes and return; mapping still consumes valid
measurements while returning. This profile does not change height.

## Simple motion workflow

The fixed schedule uses one serpentine pass, with successive targets at most
0.5 m apart, including the approach from home. It does not add a second,
perpendicular full sweep by default. Reached intermediate targets preserve
the velocity limiter; they do not command a deliberate stop. The immediate
follower target is roughly 0.2 m ahead and capped at 0.5 m from the current pose.

A* replans against measured occupancy with a 0.25 m obstacle margin. The
four-sensor reactive correction operates between replans, with warning at
0.50 m. The sweep stays 0.60 m inside the configured boundary. Unknown cells
are penalized during exploration; visiting a target does not prove the
surrounding area free. Blocked targets are deferred and retried once. One
bounded repair pass can select nearby unknown sectors adjacent to free cells.

The terminal reports MOVE, RETREAT, HOLD or RETURN, the current local target,
observed percentage and visited/remaining target counts. The survey budget
includes motion and replanning; return has its own waypoint timeout. A
budget of 180 s does not guarantee complete coverage. Choose flight duration
using measured battery endurance and reserve time for return/landing.

## What RViz means

| Geometry | Meaning |
|---|---|
| Light blue | Unknown: no accepted occupancy evidence |
| White | Measured free beam segments, or the centre cells actually traversed |
| Filled black | Repeated occupied returns |
| Orange points | Recent local obstacle returns |
| Blue line | Entire measured EKF trajectory, retained for the session |
| Purple point / short line | Nearby movement target / next route points |
| Gold disk | Home at start-relative (0,0) |
| Navy rectangle | Configured 5 × 3 m survey limits |
| Four colored rays | Valid current measurements, clipped to the mapping radius |

Pending targets and red X diagnostics are no longer drawn over the map.
Their coordinates/reasons are kept in the JSON recording. The scene is flat,
with no distorted world-space text. Console status distinguishes LIVE from
STALE DATA. After landing, the map and trajectory remain visible.

White space need not coincide with the blue line: sensors observe sideways
from the aircraft. However, observations are now restricted to a local 0.80 m
radius, not arbitrary lines extending to the 4 m sensor limit. A valid return
beyond 0.80 m clears only the near beam segment; the radius edge is not an
obstacle. Values such as 32766 do not clear space. Only the actually visited
centre cell is inferred free from pose; no circular visibility disk is invented.

This produces evidence-based fog of war. Rotating rays can reveal sectors as
the vehicle moves, but missed returns must leave unknown gaps. Four narrow
rays cannot guarantee full visibility of a disk. White triangle tiles replace
cube tiles to avoid black rendering seams. Free-space evidence can overturn
previous occupancy; time alone does not erase the map during the session.
A new session resets it to avoid mixing different EKF frames.

The presentation clips cells to the rectangle. Nearby outside obstacles may
still affect planning. Confirmed occupancy requires four returns spanning
0.6 s; it is a confidence filter, not a static/dynamic object classifier.
The conservative raw export retains occupied evidence even before visual
confirmation. Flow drift, sparse ranging and altitude-specific occlusion
remain physical accuracy limits.

## Offline handoff to CF1

Each normal navigation end writes files beside its JSONL recording:

- `cf2_flight_ID_map.json`: raw grid, confirmed presentation, trajectory,
  start, session, height and target outcome diagnostics.
- `cf2_flight_ID_map_z0.30.pgm`: conservative grayscale occupancy image.
- `cf2_flight_ID_map_z0.30.yaml`: ROS/Nav2 map-server metadata.
- `cf2_flight_ID_map_handoff.json`: coordinate and validation contract.

The PGM/YAML pair uses the ROS map-server trinary format. Image pixels are
black=occupied, white=free, gray=unknown. Resolution is 0.05 m/cell. The image
is cropped to complete cells inside the rectangle. YAML origin is measured
relative to CF2's home (0,0), keeping the session's EKF XY axis orientation.
The image's top row corresponds to the highest map Y. No-return cells remain
unknown, and separate heights are never merged into a supposedly free volume.

To export an existing snapshot manually (does not fly or overwrite files):

```bash
.venv-cf2/bin/python scripts/cf2_export_map.py data/runs/cf2_flight_ID_map.json
```

Copy the PGM, YAML and handoff JSON together to Persona 1. These are loadable
map files, not ready-to-fly global coordinates. `shared_frame_aligned=false`
and `map_validated_for_drone1=false` remain explicit. CF1 needs a measured
translation/yaw transform to its Lighthouse frame, obstacle/pose accuracy
validation and vehicle clearance. Unknown space must not be treated as free
for CF1. The original JSONL is retained for diagnosing bad measurements.

The format follows [Nav2 map server](https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/README.md)
and its [map I/O implementation](https://api.nav2.org/nav2-jazzy/html/map__io_8cpp_source.html).

## What happened in the previous flight

The recorded session `dcb2bd666a71` visited 17 of 123 targets. All 106
unresolved entries had reason `survey_budget`; its red X markers were pending
work, not detected obstacles. It used only height 0.30 m, so it could not
change Z. About 97% of lateral samples were unusable. Stationary SCAN occupied
247 of 367 logged navigation states, substantially reducing travel time.
The current simultaneous-motion profile removes that pause overhead and
uses one full sweep rather than two. It cannot repair missing sensor returns
by changing visualization colors.

## Verification

Tests cover four-direction avoidance, A* obstacle routing, home return,
continuous rotation while translating, capped nearby targets, local ray
clipping without fake boundary obstacles, persistent trails, isolated height
layers, image row orientation, YAML origin and explicit CF1 validation flags.
ROS markers and synthetic live UDP are checked without opening a radio.
Software models do not simulate aircraft dynamics, radio delay, floor
tracking failures or prove physical map accuracy. No flight is started by
the coding agent.

Multiple height support remains available for later work; this demonstration
intentionally configures a single layer only.

The ideal single-height rotating wall model completed 49/49 targets and
returned in 167.8 s with 90.5% observed cells. Adding a fixed cylinder produced
47/49 targets, 90.1% observed cells and a return in 193.4 s (including the
separate return interval); minimum ideal surface clearance was 0.275 m.
Neither scenario includes real missed returns, drift or radio/aircraft dynamics.
