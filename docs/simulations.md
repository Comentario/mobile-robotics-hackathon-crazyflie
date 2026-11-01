# Simulation and test plan

What we simulate, with which tool and how a scenario is decided to pass. It covers the T01–T18 test matrix of [roadmap.md](roadmap.md#minimum-test-matrix) and the cooperative scenario C01.

## Four levels

| Level | What it is | What it validates | What it does **not** validate | Priority |
|---|---|---|---|---|
| **0 — Unit tests** | pytest over the pure-logic modules (`ros2_ws/src/*/test/`). | Isolated algorithms: range filter, health, A*, avoidance, safety, geofence, map, metrics. | Integration, timing, ROS. | P0 |
| **1 — Our own simulator** | `simulation_adapter`: kinematic model + 2D world with raycasts of the 5 rangers + fault injection. | Full end-to-end stack, scenarios T01–T16. | Real Crazyflie dynamics and the Crazyswarm2 API. | **P0, the main one** |
| **2 — Crazyswarm2 `backend:=sim`** | Real firmware in software-in-the-loop, with `cf_adapter`. | Command mapping (takeoff, setpoints, land), topic names, firmware watchdog. | Ranges: there is no simulated Multi-ranger. | P0 (short) |
| **3 — Rosbag replay** | `ros2 bag play` of recorded runs. | Offline diagnosis, regressions (T18). | Nothing new; it replays what was recorded. | P1 |

**Why not Gazebo or Webots?** Building the model with the Multi-ranger takes days and adds dependencies. For 2D navigation with five rays level 1 is enough: fast, deterministic and with injectable faults. It stays a stretch goal if level 1 falls short.

## How to run

```bash
# Build (from the repository root, in WSL)
./scripts/build.sh
source ros2_ws/install/setup.bash

# Level 0
cd ros2_ws && colcon test && colcon test-result --verbose

# Level 1: one scenario with RViz
ros2 launch hackathon_bringup simulation.launch.py scenario:=T05_popup_obstacle rviz:=true

# Level 1: one scenario with pass/fail evaluation
ros2 launch hackathon_bringup simulation.launch.py scenario:=T09_range_dropout evaluate:=true

# Level 1: the whole matrix (results in /tmp/hk_runs/results.csv)
./scripts/run_all_scenarios.sh

# Level 2: Crazyswarm2 firmware SIL (two terminals)
ros2 launch crazyflie launch.py backend:=sim
ros2 launch hackathon_bringup real.launch.py params:=simulation.yaml overlay:=cs2_sim.yaml mission:=missions/single.yaml

# Level 3: record and replay
./scripts/record_bag.sh T05
ros2 bag play data/bags/<folder>
```

## How a scenario is evaluated

`evaluate:=true` starts `hackathon_tests/run_scenario`. `telemetry` accumulates the metrics of
the run and publishes them on `/hk/metrics`: final mission state, waypoints, collisions and
minimum obstacle clearance (against the simulator ground truth, `/hk/sim/ground_truth`, and
the world boxes of `/hk/sim/obstacles`), the safety/health/planner states seen, `max |x|`,
`max |y|`, `max z` and the final `z`. When the mission ends (COMPLETED/ABORTED, plus a few
settle seconds so the landing is counted) or the scenario `timeout` expires, the runner
compares the metrics with `expected`, prints PASS/FAIL, appends a row to
`<output_dir>/results.csv` and exits with code 0/1, which shuts the launch down.
`telemetry` also writes a JSON summary per run to `output_dir`.

## Scenario structure

A scenario is world + mission + faults + expected result. Files in [`ros2_ws/src/hackathon_bringup/config/`](../ros2_ws/src/hackathon_bringup/config/):

```yaml
# config/scenarios/T09_range_dropout.yaml
world: worlds/wall.yaml          # obstacles (simulation_adapter/world.py)
mission: missions/line.yaml      # waypoints
start: [-1.5, 0.0, 0.0, 0.0]     # initial x, y, z, yaw
timeout: 60.0
faults:                          # simulation_adapter/faults.py
  - {type: range_dropout, sensor: all, t_start: 5.0, t_end: 8.0}
expected:                        # hackathon_tests/scenario.py
  max_collisions: 0
  must_see_safety: [SLOW, HOVER]
```

Available faults: `range_spike`, `range_dropout`, `pose_delay`, `pose_freeze`, `pose_jump`, `pose_dropout`, `command_pause`.

`expected` keys: `mission_state`, `max_collisions`, `min_obstacle_distance`, `must_see_safety`, `must_see_health`, `must_see_planner`, `max_abs_xy`, `max_z`, `final_z_max`. Collisions and the minimum distance are computed against the real simulator pose (`/hk/sim/ground_truth`), not against the estimated one. `must_see_*` lists alternatives: at least one of the listed states must be observed.

To add a scenario, copy an existing one, change world, faults and `expected`, and add a row to the following table.

## Scenario catalogue

| Scenario | Test | World | Fault | Checks | Needs | Priority |
|---|---|---|---|---|---|---|
| `T01_single_waypoint` | T01 | empty | — | Reaches the waypoint | VS-1 | P0 |
| `T02_multi_waypoint` | T02 | empty | — | Follows the waypoints in order | VS-1 | P0 |
| `T03_static_wall` | T03 | wall | — | Goes around the wall | A* (`planner_mode: astar`) | P1 |
| `T04_corridor` | T04 | corridor | — | Margin in the corridor | Map + A* | P1 |
| `T05_popup_obstacle` | T05 | popup | — | Slow/hover without collision | Reactive avoidance | P0 |
| `T06_vanishing_obstacle` | T06 | vanishing | — | The decay frees the route | Map with decay | P1 |
| `T07_approaching_obstacle` | T07 | approaching | — | The TTC brakes | d_dot / TTC | P1 |
| `T08_range_spike` | T08 | empty | `range_spike` | The spike does not stop the mission | Range filter | P0 |
| `T09_range_dropout` | T09 | wall | `range_dropout` | Degrades without crashing | Range watchdog | P0 |
| `T10_pose_delay` | T10 | empty | `pose_delay` | Health DEGRADED/LOST | Health monitor | P0 |
| `T11_pose_freeze` | T11 | empty | `pose_freeze` | LOST → hover | Health + safety | P0 |
| `T12_localization_lost` | T12 | empty | `pose_dropout` | Hover → land | Safety (`lost_policy`) | P0 |
| `T13_no_path` | T13 | blocked_goal | — | NO_PATH, does not advance blindly | A* | P1 |
| `T14_geofence` | T14 | empty | — | Setpoint clamped | Geofence | P0 |
| `T15_altitude_envelope` | T15 | low_ceiling | — | Exceeds neither z_max nor the ceiling | Envelope + `up_range` | P0 |
| `T16_command_stop` | T16 | empty | `command_pause` | Watchdog / hover | Simulator watchdog | P0 |
| — | T17 | — | Kill a node | See manual procedure | — | P1 |
| — | T18 | — | — | Rosbag replay | — | P1 |
| `C01_cooperative_route` | C01 | cooperative_room | — | CF1 flies the CF2-planned route | `route` key, `simulation_cf.launch.py` | P0 |

`simulation.yaml` uses `planner_mode: astar` with the `local_world_model` map, so the P1
scenarios run with the full planner. C01 uses the Crazyswarm2-API path
(`simulation_cf.launch.py`) and is checked with `scripts/verify_mvp_integration.py`; it is
not part of `run_all_scenarios.sh`.

## Manual procedures

**T17 — node restart.** With `T02_multi_waypoint` running, run `pkill -f local_navigator_node`. Expected: without `/hk/cmd/nav`, safety_supervisor detects `command_timeout` → HOVER. Restart the node with `ros2 run local_navigator local_navigator_node --ros-args --params-file ...` and check that the mission continues or aborts in a controlled way. Repeat with `state_monitor` (→ LOST) and with `safety_supervisor` (→ adapter watchdog: hover).

**T18 — replay.** Record `T05_popup_obstacle` with `scripts/record_bag.sh`, replay the bag without the simulator and check in RViz/PlotJuggler that the obstacle moment, the `safety_state` change and the replan are visible.

## Minimum fidelity of our own simulator

For the level 1 results to be meaningful:

- **Ranges:** raycasts against walls, ceiling, floor and obstacles; Gaussian noise; `+inf` beyond `range_max`; rate ~20 Hz.
- **Pose:** small noise, rate ~50 Hz; faults are applied **only to the published pose**, never to the real pose.
- **Dynamics:** speed and acceleration limits; first-order response to the setpoint.
- **Watchdog:** if no setpoints arrive for `command_watchdog`, hover. Adjust to the real firmware behaviour once confirmed (TP-08).
- **Collision:** `telemetry` counts every new contact of the real pose with an obstacle or wall; the scenario fails with `max_collisions: 0`.

## Integrated MVP of the three persons (branch integration-mvp-crazyswarm)

The cf_adapter of main is kept, the perception and safety of person 2 are added, and the
mission/navigation of person 3. All of them use the ROS clock for data and control;
`time.monotonic()` only bounds the waits of the test script. In SIL they all consume `/clock`.
The navigator proposes an XY velocity with the waypoint height; cf_adapter generates FullState.
`/hk/safety_state` communicates the state; `/hk/cmd/safe` communicates commands: they are
different messages.

On Ubuntu 24.04/ROS Jazzy, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/build.sh
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=93
export ROS_LOCALHOST_ONLY=1
python3 scripts/verify_mvp_integration.py
python3 scripts/verify_mvp_integration.py --scenario T02_multi_waypoint
python3 scripts/verify_mvp_integration.py --scenario C01_cooperative_route
```

These tests start and stop the nodes, wait for normal health/safety, start the mission
through the service and check arrival, landing, frames, the observed topics and a single
publisher. Logs in `/tmp/hk_mvp_emulator_*.log`. To watch the system and open rqt_graph/RViz
separately:

```bash
ros2 launch hackathon_bringup simulation_cf.launch.py backend:=emulator scenario:=T01_single_waypoint
ros2 launch hackathon_bringup simulation_cf.launch.py scenario:=C01_cooperative_route rviz:=true
```

### Crazyswarm2 SIL with synthetic sensors

It requires `crazyflie_sim`, the `np` backend and `cffirmware` bindings built for the
Python/ROS version of that machine. Do not copy Humble binaries to Jazzy.
In a first terminal, after sourcing ROS and the workspace, and exporting the same domain:

```bash
# If cffirmware is not installed in Python, add its build directory to PYTHONPATH.
python3 scripts/run_crazyswarm_sil.py --scenario T01_single_waypoint
```

The script uses the installed server and its URDF description, enables only cf231 and TF,
sets the initial position of the scenario and publishes /clock. In another terminal with
the same environment:

```bash
python3 scripts/verify_mvp_integration.py --backend sil --timeout 300
# Or a visible run with autostart:
ros2 launch hackathon_bringup simulation_cf.launch.py backend:=sil scenario:=T01_single_waypoint
```

Use the same scenario in both terminals. For T02 restart the server with
`--scenario T02_multi_waypoint` and pass that argument to the verifier as well.
The installed SIL simulation does not implement emergency nor notify_setpoints_stop:
the mission test certifies neither those services nor the real watchdog.
Final compatibility with ROS Jazzy must be validated on the machine of person 2.

Local validation (ROS Humble, 2026-10-08): 14 packages built; 147 tests passed and
19 pending/skipped; T01 and T02 completed with the emulator, T01 completed with Crazyswarm2
SIL. This does not replace running on Jazzy. The current results are in the
[README](../README.md#verification).
