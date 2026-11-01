# 2D mapping during the drone 2 flight

**For navigation with avoidance and return, use the newer
[CF2 autonomous guide](cf2_autonomous_demo.md).** The flight commands in this
document without `--navigate` are open-loop mapping tests.

A single flight produces telemetry, an incremental map and the visualization. There is
no need to land to compute the map. It does not use Lighthouse nor connect to drone 1.

## Terminal 1: map and ROS publication

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ..
.venv-cf2/bin/python scripts/cf2_live_map.py --ros
```

Open **http://localhost:8765** in the Windows or Ubuntu browser. Keep this terminal
open. Without `--ros`, the browser works without loading ROS. The process has no radio
connection and cannot send commands to the drone.

## Terminal 2: flight and live streaming

Close cfclient and any server that uses the same radio. Prepare a clear area, obstacles
outside the trajectory and a textured surface. The flight is open loop: the script has no
obstacle avoidance and no automatic cut-offs, at the operator's request. The previous
landing had incidents whose cause was not determined; have the mentor present during
the run.

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --height 0.3 --hover-seconds 3 \
  --square-side 0.2 --live-map
```

In one flight: climb, hover, four 20 cm legs (front → left → back → right), a 1 s pause
after every leg and landing. Speed 0.1 m/s. Directions are relative to the drone, with no
programmed yaw turns. Velocity integration does not guarantee that the square closes
physically. Alternatively, `--forward 0.2 --live-map` keeps the already rehearsed forward
flight. Do not combine `--forward` and `--square-side`.

For more coverage, `--square-side 0.5` requests 50 cm sides. `--rectangle 0.5 0.3`
requests front 50 cm → left 30 cm → back 50 cm → right 30 cm. Do not combine routes.
Keep the whole footprint of the flight clear: chairs outside the perimeter and margin for
the drone size/drift. With a front return of about 60 cm in previous rehearsals, do not
fly 50 cm forward from the same position towards that chair; rearrange the space first.

## Data and visualization

- Telemetry at 10 Hz over local UDP (port 8766); the browser and ROS update at 5 Hz.
- 8 × 8 m map, 5 cm cells. XY pose in metres, yaw in degrees, original ranges in
  millimetres. Timestamp association up to 50 ms.
- Only integrate the airborne window `hovering=true` and an estimated height of
  0.2–0.4 m. Do not use ground samples, the initial EKF reset or the landing for mapping.
- Every lateral return of 50–4000 mm marks free space up to the obstacle and occupied at
  the end. Invalid ranges/32766 leave unknown space.
- Observations can be corrected by new rays and expire after 120 s by default
  (`--decay-seconds` changes it): it is a temporary local map, not a permanent map or SLAM.
- A box held by a person represents a moving object. It can be seen live, but it does not
  validate the position of a fixed wall. Use walls or boxes resting on the floor to validate
  the static map. Do not require anyone to hold an object still.
- A new flight has its own session and frame; do not merge different origins.
- If the telemetry stops, the page shows it. The drone receives no commands from the map
  nor from the browser. Enter is not an emergency stop.
- Every flight keeps a JSONL in `data/runs/cf2_flight_<session>.jsonl`.

## Interface for Persons 1 and 2

With `--ros`, it publishes:

| Topic | Type | Content |
|---|---|---|
| `/cf2/map` | `nav_msgs/msg/OccupancyGrid` | -1 unknown, 0 free, 100 occupied |
| `/cf2/odom` | `nav_msgs/msg/Odometry` | Local pose with yaw; provisional covariance, not the EKF one |
| `/cf2/ranges` | `hackathon_interfaces/msg/MultiRange` | Six ranges in metres; invalid return as NaN |
| `/cf2/range/front`, `/left`, `/back`, `/right` | `sensor_msgs/msg/Range` | Only usable returns, in metres |

Map frame: `cf2_local_<session>`. The sensor frames belong to the drone; their TF
transforms and a transform to the Lighthouse frame of drone 1 are not published yet.
Before using it for shared navigation, align both frames and measure the uncertainty
(the offline route is `scripts/plan_shared_map.py`, see [the coverage demo](cf2_coverage_demo.md)).
Invalid ranges do not publish a new sample: consumers must check the age.
The topics are reachable by other ROS nodes in the same environment; communication
between WSL/Ubuntu laptops requires checking DDS discovery, the network and ROS_DOMAIN_ID.

This is the hardware route of drone 2. `real.launch.py` belongs to the Crazyswarm2 stack
and does not replace these two processes. Do not start it at the same time against the
same radio. The planning TPs are integrated in the branch, but they do not command the
flight script yet. The local map must not be renamed to the Lighthouse `map` without a
measured transform.

The HTTP interface `GET http://localhost:8765/state` delivers the same map, pose, ranges,
trajectory, session, data age and statistics as JSON. It is served only on loopback; it is
not exposed to other machines.

## Verification without flight

```bash
.venv-cf2/bin/python -m unittest discover -s scripts -p 'test_cf2_*.py' -v
.venv-cf2/bin/python scripts/cf2_live_map.py \
  --replay data/runs/cf2_flight_9de3a2898a4a.jsonl
```

Replay reproduces recorded data at its original pace, without a radio connection.
The age indicator shows when the replay has ended.
