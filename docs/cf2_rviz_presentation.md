# CF2 live perception in RViz

English presentation, top-down metric view, light background. RViz receives
ROS messages only; it does not connect to the radio or control the drone.
Close the previous mapper before starting: one process owns UDP port 8766.

## Start the presentation

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
bash scripts/cf2_view_live.sh
```

This sources ROS and the workspace, starts the mapper with ROS publication,
and opens RViz with the saved configuration. No browser is required. Close
RViz or press Ctrl+C in the terminal to end this viewer.

In a separate terminal, run the existing `cf2_first_hover.py --fly --navigate
--live-map` command. Keep the accepted flight duration and speed. Speed units
are **m/s**: `--speed 0.2` means 20 cm/s, not 0.2 cm/s.

## Read the scene

| Element | Meaning |
|---|---|
| White cells / gray background | Observed free rays / unknown space |
| Dark navy cells | Confirmed obstacle returns |
| Orange dots | Recent raw hits, retained 1.2 s |
| Cyan line | Actual estimated XY flight trajectory |
| Purple line | Planned route; empty during reactive hover without goals |
| Gold disk | Start (0,0) of this session |
| Blue body/red arrow | Drone position and heading; body turns gray when stale |
| Green/cyan/amber/purple rays | Front/left/back/right usable ToF readings |
| Viewer terminal | Live control state, freshness and relative XYZ position |

The metric grid uses 0.5 m spacing. Drag/zoom with the RViz Move Camera tool.
The default scene renders observed cells as markers. The optional occupancy
texture display is disabled because its indexed-image shader failed under
WSLg OpenGL during replay. `/cf2/presentation/map` remains published. The
marker/trajectory RViz subscriptions were verified without that shader error.
The saved camera uses Orbit looking vertically downward, with an explicit
7.3 m distance and focal point at the session origin. The launcher uses software
OpenGL and XCB for WSL compatibility; set `CF2_RVIZ_SOFTWARE=0` to try GPU rendering.
The RViz root and each display explicitly include their saved `Value` state.

The dedicated `cf2_display` frame subtracts the session start XY. It has no
transform to Lighthouse. Topics under `/cf2/presentation/*` are explicitly
for display, not a global navigation frame for drone 1.

## Stable observations without invented geometry

A moving person creates moving returns. The presentation confirms an occupied
cell only after at least four hits spanning 0.6 s, clears it when free rays
overturn the occupancy evidence, and retains confirmed returns throughout the session by default.
Transient hits are shown separately rather than accumulating as permanent
walls. Repetition is not proof that an object is static: a person who pauses
can still appear in the persistent layer. The controller and `/cf2/map` keep
their original observations; visual filtering cannot hide a hazard from control.

For a presentable environmental map, demonstrate with a fixed chair, panel or
box at sensor height. Keep people out of the measured scene during that scan.
Four narrow range directions and Flow drift do not reconstruct continuous
walls like a scanning lidar or SLAM system. This display does not fill gaps,
straighten unmeasured walls, or claim unseen space is free.

## Replay without flight

```bash
bash scripts/cf2_view_rviz.sh --replay data/runs/cf2_flight_SESSION.jsonl
```

Use the recording path printed by the flight script. Replay never arms the
drone. Once recorded data ends, the viewer terminal shows STALE DATA and live rays
disappear; the trajectory and free observations remain for review, and obstacle
observations remain until contradicted by measured free space (or until the session changes).

## Verification

Presentation messages update at 10 Hz; the ROS-independent flight control
continues at its existing rate. RViz startup and topic subscriptions were checked in WSL. Automated window
capture returned black under WSLg, so visible rendering still needs confirmation
in the user session; successful startup alone does not verify the image. Map/navigation tests cover confirmation and expiry independent
of the control map. A ROS publisher/subscriber check validates presentation
grid, marker geometry, frame, origin and front ray. RViz is installed in WSL;
Future GUI sessions require WSLg/OpenGL to remain available on the laptop.

## Blank scene recovery

Close the old RViz and stop its mapper with Ctrl+C. Do not save the old RViz
configuration over the corrected file. First replay a known recording:

```bash
bash scripts/cf2_view_rviz.sh --replay data/runs/cf2_flight_2afbd88b8163.jsonl
```

Allow 10 seconds for the recorded takeoff. The grid should be visible immediately;
then the start marker, trajectory and sensor rays appear. If the scene remains
blank, expand the marker display and inspect its Status, and check the viewer
terminal for errors. Use this replay before another physical flight.

## Current flight, exclusively live

Close every previous RViz/mapper terminal before starting. The live-only launcher
rejects replay input and uses ROS domain 43 by default to isolate its presentation
from older publishers. Override with `CF2_RVIZ_DOMAIN_ID` when needed.

Terminal 1:

```bash
bash scripts/cf2_view_live.sh
```

Terminal 2 (same accepted reactive flight, no added route):

```bash
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --navigate --live-map \
  --height 0.3 --speed 0.2 --hover-seconds 60
```

The viewer terminal prints input freshness, current XY, mapped samples and trail
length every two seconds. `WAITING FOR TELEMETRY` before connecting is normal.
During the flight it must show `LIVE` and increasing counters. `STALE DATA`
means fresh pose/range telemetry has stopped, not that the scene is current.
The four sensor rays, drone position, trajectory and cells update at 10 Hz.
A static map may stay unchanged in an unchanged environment; the live rays and
position reflect current readings. The viewer itself never opens the radio.

For the current moving survey, use [the coverage mission guide](cf2_coverage_demo.md)
instead of the reactive-hover command above.

## Persistent height layers

See [the live coverage guide](cf2_coverage_demo.md#discrete-height-sweeps-and-persistent-fog-of-war)
for the current commands, height bins and fixed color palette. The light-blue
survey rectangle denotes unknown space, white denotes measured free space,
and black denotes confirmed occupancy. Colored contours distinguish heights.
The full flight trail is retained for the session. Presentation is clipped to
the configured survey rectangle. Each height is also published separately at
`/cf2/presentation/layer_N`. The combined view is for presentation; CF1 must
use a validated height layer with a measured frame alignment.

## Current one-height live profile

Follow [the current coverage guide](cf2_coverage_demo.md). The active profile
rotates WHILE translating; it no longer pauses for stationary SCAN. Mapping
uses a local 0.80 m radius and valid measurements only. Red X / pending target
markers are removed from the scene, while their diagnostics remain recorded.
The exporter writes a cropped conservative PGM/YAML pair plus a handoff JSON
for offline exchange with CF1. Shared coordinates still need measured alignment.

## Adaptive coverage status

The live launcher opens an English Qt status window beside RViz. It shows
measured observed percentage, EXPLORE/INSPECT/RETURN phase and stop trigger.
This avoids distorted RViz billboard text. The map, actual trajectory and
current sensor rays remain live ROS displays in RViz. The flight guide
`cf2_coverage_demo.md` documents the one-height adaptive profile.
