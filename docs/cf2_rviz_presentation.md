# CF2 live perception in RViz

English presentation, top-down metric view, dark background. RViz receives
ROS messages only; it does not connect to the radio or control the drone.
Close the previous mapper before starting: one process owns UDP port 8766.

## Start the presentation

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
bash scripts/cf2_view_rviz.sh
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
| White/gray occupancy | Observed free rays / unknown space |
| Blue cells | Repeated, recently observed obstacle returns |
| Orange dots | Recent raw hits, retained 1.2 s |
| Cyan line | Actual estimated XY flight trajectory |
| Purple line | Planned route; empty during reactive hover without goals |
| Gold disk | Start (0,0) of this session |
| White body/red arrow | Drone position and heading |
| Green/cyan/amber/purple rays | Front/left/back/right usable ToF readings |
| Robot label | Live control state and relative XYZ position |

The metric grid uses 0.5 m spacing. Drag/zoom with the RViz Move Camera tool.
Choose Orbit in Views if a perspective view is preferable. The saved default
is top-down to make 2D spatial relationships and distances easy to judge.

The dedicated `cf2_display` frame subtracts the session start XY. It has no
transform to Lighthouse. Topics under `/cf2/presentation/*` are explicitly
for display, not a global navigation frame for drone 1.

## Stable observations without invented geometry

A moving person creates moving returns. The presentation confirms an occupied
cell only after at least four hits spanning 0.6 s, clears it when free rays
overturn the occupancy evidence, and hides it after 5 s without another hit.
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
drone. Once recorded data ends, the robot label shows STALE DATA and live rays
disappear; the trajectory and free observations remain for review, and obstacle
observations expire according to their lifetimes.

## Verification

Presentation messages update at 10 Hz; the ROS-independent flight control
continues at its existing rate. RViz startup with this configuration and OpenGL
rendering was verified in WSL. 21 map/navigation tests pass, including confirmation and expiry independent
of the control map. A ROS publisher/subscriber check validates presentation
grid, marker geometry, frame, origin and front ray. RViz is installed in WSL;
Future GUI sessions require WSLg/OpenGL to remain available on the laptop.
