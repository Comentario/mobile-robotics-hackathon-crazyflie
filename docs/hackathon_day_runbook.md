# Event runbook — TP-42 E

Template prepared on 8 October 2026. It does not certify a real flight. Use a common
revision of the repository and fill in the data of the delivered unit.

## Owners and files

- Person 1: configuration, launch, recording and log of every rehearsal.
- Person 2: frames, localization health, ranges and safety supervisor.
- Person 3: mission, direct route, tracking and waypoints.
- Test operator: one designated person who knows the available stop; agree with the
  mentors on the rehearsal space and the emergency procedure.

| File | Use | Confirm before hardware |
|---|---|---|
| `config/event.yaml` | Stack parameters | Robot, frames, limits, heights, thresholds, speeds, map and tolerances |
| `config/event_hardware.yaml` | Crazyswarm2 template | URI, robot/type, firmware, estimator, decks, logging and `enabled` |
| `config/missions/event.yaml` | Waypoints in map | Origin/axes, order, heights and route inside the geofence |
| `config/frames.yaml` | Sensor TF | Physical offsets; today they are nominal |

The config paths are relative to `ros2_ws/src/hackathon_bringup`. The current numeric
values are development values. The URI belongs to Crazyswarm2; `cf_adapter.robot_name`
must exactly match the `robots` key. The `EVENT_TYPE_TODO` block is a template: replace it
with the configuration of the received unit. It configures neither Lighthouse/Flow nor
replaces the calibration.

## 1. Preparation and check without hardware

From the root, in every ROS terminal:

```bash
source /opt/ros/humble/setup.bash
bash scripts/build.sh
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1
```

Domain 92 is an example for tests on a single machine. For several machines, agree on the
domain and the network configuration; `ROS_LOCALHOST_ONLY=1` prevents talking to other
machines. Do not share a domain with other people's tests.

```bash
/usr/bin/python3 scripts/verify_event_config.py
bash scripts/verify_tp10.sh
/usr/bin/python3 scripts/verify_tp12.py
```

The first command must report static consistency and list the placeholders. The automatic
tests do not require a radio. TP-12 tests start-up/connections; at the time of writing
T01/T02 did not count as an approved mission until navigation and safety were integrated
(they now pass end to end in simulation, see [simulations.md](simulations.md)).

## 2. When the drone is received

1. Confirm the URI, robot id/type and firmware revision with the mentors.
2. Check the Crazyradio, USB permissions, battery and the decks actually mounted.
   Crazyradio/native Ubuntu belong to TP-42 A/B; record anything pending.
3. Confirm how XYZ/yaw is obtained: Lighthouse, Flow or another source. Verify the pose by
   moving the drone by hand, signs and units, without commanding a takeoff.
4. Confirm the global frame, origin, orientation and limits of the real space.
   `source_frame` and `map_frame` must share origin/axes: the adapter does not calibrate
   between them. The limits include the floor Z=0 for landing.
5. Verify the five Multi-ranger rays and the bottom ToF if available.
   For vertical logging, check `range.up`/`range.zrange` in the firmware, enable the
   commented block and configure the real topic in `event.yaml`.
   Without vertical logging the adapter publishes NaN; do not interpret NaN as free space.
6. First run the official procedure/example indicated by the mentors.
   The presence of an endpoint does not prove its effect (see TP-08).

Update the three YAML files, the sensor offsets and the delivered estimator configuration.
Keep `mission_manager.autostart=false`. Tune the envelope and the speeds together with
persons 2/3; do not disable the required ranges to hide an incomplete integration.

```bash
/usr/bin/python3 scripts/verify_event_config.py --ready
```

It must exit with code 0 and without placeholders. This checks files, not the calibration
nor the firmware behaviour. Record the Git revision and the confirmed values; the example
waypoints still need approval.

## 3. Starting the unit and the stack

In two terminals, with ROS sourced and the same domain:

```bash
# Terminal A: external server. Run it only after completing event_hardware.yaml.
EVENT_CONFIG="$PWD/ros2_ws/src/hackathon_bringup/config"
ros2 launch crazyflie launch.py backend:=cflib \
  crazyflies_yaml_file:="$EVENT_CONFIG/event_hardware.yaml" \
  teleop:=False mocap:=False rviz:=False
```

This example uses the installed cflib backend and no external mocap. If the mentors
require mocap, another backend or a server/estimator configuration, use their procedure.
Do not change firmware parameters blindly.

```bash
# Terminal B: our stack, without starting the mission automatically.
EVENT_CONFIG="$PWD/ros2_ws/src/hackathon_bringup/config"
ros2 launch hackathon_bringup real.launch.py \
  params:="$EVENT_CONFIG/event.yaml" \
  mission:="$EVENT_CONFIG/missions/event.yaml"
```

A single-command `event.launch.py` belongs to TP-42 D and is pending.
Do not start `simulation_adapter` and `cf_adapter` together in the same domain.

## 4. Checks before starting the mission

```bash
ros2 node list
ros2 topic echo --once /hk/state
ros2 topic echo --once /hk/adapter/ranges
ros2 topic echo --once /hk/localization_health
ros2 topic echo --once /hk/safety_state
ros2 topic info /hk/cmd/safe --verbose
ros2 service list
```

Confirm recent timestamps, consistent XYZ/yaw and rates sufficient for the configured
timeouts; if an echo waits forever, cancel it and fix the source. Health GOOD, supervisor
without fault, required ranges valid and a single publisher of `/hk/cmd/safe`:
`safety_supervisor`. Confirm the connections
`/hk/goal → /hk/path → /hk/path/processed → /hk/cmd/nav → /hk/cmd/safe`.
takeoff/land/emergency and mission/start/abort services available. Do not publish
synthetic commands directly to `/hk/cmd/safe` on hardware.

If colleagues' nodes or inputs are missing, do not start the mission. The simulation
verifies neither the firmware watchdog nor the real behaviour of emergency.

## 5. Rehearsals and logging

Check in simulation first: one waypoint (T01), several (T02), and then obstacles/faults
once their TPs are implemented. A launch without errors is no evidence of reaching
waypoints nor of avoidance.

On hardware, advance in stages following the mentors' procedure: takeoff and hover, T01,
T02, static obstacle, replanning and dynamic obstacles only if those capabilities are
implemented and verified. Repeat a failed stage before going further.

```bash
# Terminal C: start before the rehearsal; stop with Ctrl+C after landing.
bash scripts/record_bag.sh T01_event
# Operator terminal: start the mission only after the previous checks.
ros2 service call /hk/mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /hk/mission_status
```

`success=true` in takeoff/land confirms acceptance, not the end of the manoeuvre; check the
height and the ground contact. Record rehearsal, commit, robot/firmware, YAML files used,
result, time, intervention and bag path. Copy bags and configuration to persistent storage;
`/tmp/hk_runs` is not a final archive.

## 6. Stop and recovery

- Mission abort: `/hk/mission/abort` requests to finish/land according to its logic.
  If it does not respond, use the stop procedure agreed with the mentors.
- Direct landing available: `/hk/land`; confirm the descent and the ground.
- `/hk/emergency` is a last-resort cut-off and may cause a fall; its effect depends on the
  backend/firmware. In our simulator it stays latched until restart.
- Ctrl+C on the launch closes all nodes; it is not a landing command.
  Land and verify first, then close the bag, the stack and the server.

```bash
ros2 service call /hk/mission/abort std_srvs/srv/Trigger '{}'
# If a landing request to the adapter is appropriate:
ros2 service call /hk/land hackathon_interfaces/srv/Land '{duration: 2.0}'
```

On a lost pose, invalid ranges, stale commands or radio errors: stop the rehearsal with the
tested procedure, save the logs, fix the cause and go back to the previous checks. Do not
assume hover because the stream went silent. Before the next rehearsal, check the battery,
the estimator state and that no other publishers/adapters are left. Freeze the YAML files of
the configuration that passed the tests; record any later adjustment.
