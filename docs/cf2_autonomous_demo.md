# CF2: onboard sensing, local navigation and return

Track 1: [Autonomous Navigation](https://mobile-robotics.eu/) requires
waypoint navigation with onboard perception, localization, planning and
collision-free trajectory execution under environmental changes. The public
page does not specify a course, exact heights, tolerances or scoring rubric.
The mentor has not provided additional rules. This is our proposed demo,
not an official certification of track completion.

## What changed

`--navigate` connects the team's pure algorithms to the SAME cflib radio
connection used for takeoff, motion and landing. Previously the fixed square
ignored obstacle ranges. This is a direct-radio adapter, independent of the
Crazyswarm2/Lighthouse launch and ROS scheduling. The EKF and PID remain onboard.

Pipeline: telemetry -> local map -> inflated A* -> processed path -> follower
-> smoothing -> reactive avoidance -> supervisor -> body-frame velocity.

| Existing TP code reused | Role |
|---|---|
| 15, 16 | Pose freshness/jump/speed checks and localization speed policy |
| 20 | Range filter and closing speed |
| 26 | Mission lifecycle, waypoint progression, timeout |
| 27, 28 | A*, unknown-space cost |
| 30, 32, 33 | Path processing, lookahead follower, acceleration limiting |
| 34, 36 | Range repulsion, closing speed/TTC |
| 37–39 | Supervisor, stream freshness, local geofence |

The existing live mapper supplies the missing ROS world model for this adapter.
Inflation is 25 cm around observed hits; replanning occurs every second and on
waypoint change. The TP-29 ROS trigger node is not run in this adapter. Height
stays at 30 cm, yaw is not deliberately rotated. Upper clearance/height problems
request the existing landing routine; no new emergency motor-cut is introduced.
This is not the entire ROS pipeline or a declaration that all TPs through 42
are complete. General ROS world-model/telemetry/scenario skeletons remain.
The earlier ROS emulator end-to-end test is still unvalidated due to stream
gaps and timeout. Software policy tests here do not replace real flight tests.

## Start and coordinates

The first received airborne pose after MotionCommander resets the estimator
defines the mission XY origin. It is saved and displayed as **Start (0, 0)**.
All waypoint offsets are in local EKF X/Y axes (not changing body axes).
The grid and planned path retain session EKF coordinates; the display subtracts
start for the relative position. The square returns to within 8 cm of the
estimated origin. Physical return accuracy depends on Flow/EKF drift.

## First hardware validation: reactive hover

Use a textured, well-lit floor and the mentor's test area. Close cfclient and
other radio owners. Do not run `real.launch.py` alongside this script.

Terminal 1, restart the visualizer to load the updated page:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/python scripts/cf2_live_map.py
```

Open http://localhost:8765. ROS publication is optional: use `--ros` after
sourcing ROS and `ros2_ws/install/setup.bash`.

Terminal 2:

```bash
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --navigate --height 0.3 --hover-seconds 10 --live-map
```

No waypoint route yet. A visible obstacle 35–45 cm away should cause a small
horizontal correction AWAY from it. Under 25 cm, the supervisor requests
horizontal hover, not forward motion. Avoid bringing anything close to the
propellers. Confirm `CLOSED LOOP` and `NAV:` messages. This validates actual
command signs and sensor-to-actuator wiring before a path around an obstacle.

## Waypoint inspection and return

After the reactive hover behaves correctly, start in a clear area. The first
waypoint is X=1.1 m, Y=0 relative to start, followed by X=0,Y=0. These are
DEMO offsets; adapt placement to the available test area. The local geofence
extends 1.5 m in each XY direction from start. Keep enough lateral clearance
for bypassing the obstacle. Do not place the destination inside its margin.

First test the same route without an obstacle. Then place one fixed obstacle
approximately halfway toward the first waypoint, visible at flight height.
A chair at 30 cm may present only thin legs and gaps: verify range detection
on the ground at that sensor height first. A broad fixed panel on the chair
or a box is a more repeatable initial target; do not hold it by hand.

```bash
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --navigate --height 0.3 --forward 1.1 --return-home \
  --speed 0.1 --waypoint-timeout 60 --live-map
```

Expected: new black map hits, purple path bends around them, blue actual path,
waypoint progression, return toward yellow Start marker, then `NAV LAND:
mission_complete`. If no route is found for 5 seconds, request landing and
report `no_path_timeout`; do not pretend this is successful obstacle bypass.
Stale pose/ranges cause hover and, after 2 seconds, a landing request.
Physical touchdown must be observed; the landing API cannot certify it.

`--navigate --square-side 0.5` also follows four local waypoints with feedback;
without `--navigate`, the same square remains the old timed open-loop route.

## Deliverables and drone 1

The browser and ROS update during flight. JSONL records range/pose/commands/
navigation decisions. On normal navigation completion a `_map.json` snapshot
is written beside the recording, retaining session origin and observed cells.
The **Download local map** link also exports the current snapshot.

Evidence of stage 1: video showing bypass and return, map snapshot, JSONL,
mission-complete reason, no collision/intervention. The four rays do not see
every object or all heights. Unknown space remains unknown in the map; A*
penalizes it for exploration. No-return readings do not guarantee empty space.
No SLAM/global correction is supplied by Flow alone.

Drone 1 cannot safely consume these coordinates directly. Mark the start and
heading physically, measure the corresponding Lighthouse pose, and construct
an XY translation/yaw transform before transferring occupied cells. Export
explicitly marks `shared_frame_aligned=false`. Sending the JSON or ROS topics
does not perform this alignment. Shared-map control of drone 1 is stage 2.
