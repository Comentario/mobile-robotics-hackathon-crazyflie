# Coordinate frames (TP-13)

Which coordinate systems the stack uses, how they relate and with which conventions. Complements the "Frames (TF)" section of [interfaces.md](interfaces.md).

## Conventions

| Topic | Convention | Reference |
|---|---|---|
| Units | Metres, seconds and radians (SI). | Rule 5 of interfaces.md |
| `map` and `base_link` axes | **x forward, y left, z up** (right-handed). | REP-103 |
| Angles | Positive `yaw` = **counter-clockwise** turn seen from above (from x towards y). `yaw = 0` faces +x of `map`. | REP-103 |
| Range sensor axes | The **x axis of the sensor frame points along the ray**. | `sensor_msgs/Range` |
| Range values | `+inf` = no return, `-inf` = too close, `NaN` = invalid. | REP-117 |

## Frame tree

```text
map ──(state_monitor, dynamic)──> base_link ──(static, frames.yaml)──> front_range
                                                                    ├─> back_range
                                                                    ├─> left_range
                                                                    ├─> right_range
                                                                    ├─> up_range
                                                                    └─> down_tof
```

| Frame | Parent | What it represents | Who publishes it | Type |
|---|---|---|---|---|
| `map` | — | The room. Fixed origin on the floor. The whole stack works here (waypoints, map, route, setpoints). | — (root) | — |
| `base_link` | `map` | The centre of the drone. It moves with it. | `state_monitor`, from `/hk/state` | Dynamic |
| `front_range` … `up_range` | `base_link` | Each Multi-ranger sensor. | `static_transform_publisher` (launch, from `frames.yaml`) | Static |
| `down_tof` | `base_link` | Flow Deck bottom ToF (height above the floor). | Same as above | Static |

**`odom` is not used for now:** the onboard EKF pose already arrives fused in a single global frame (see interfaces.md).

## Sensor positions on the drone

Values of `hackathon_bringup/config/frames.yaml` (`xyz` in m, `rpy` in rad):

| Frame | xyz | rpy | Ray direction |
|---|---|---|---|
| `front_range` | (0.02, 0, 0.03) | (0, 0, 0) | +x of `base_link` (front) |
| `back_range` | (-0.02, 0, 0.03) | (0, 0, π) | -x (back) |
| `left_range` | (0, 0.02, 0.03) | (0, 0, π/2) | +y (left) |
| `right_range` | (0, -0.02, 0.03) | (0, 0, -π/2) | -y (right) |
| `up_range` | (0, 0, 0.04) | (0, -π/2, 0) | +z (up) |
| `down_tof` | (0, 0, -0.01) | (0, π/2, 0) | -z (down) |

The offsets are **approximate**: they must be measured on the real Crazyflie with the Multi-ranger mounted (TODO in `frames.yaml`). A 1–2 cm error is negligible compared with the obstacle inflation (`obstacle_inflation_radius: 0.25`); the **direction** of every ray, however, is critical.

## Relation with Crazyswarm2 and the hardware

- Crazyswarm2 publishes its own TF `world -> cfXXX`. In SIL mode, `sim_sensors` uses it to generate the pose and the ranges. Then `cf_adapter` converts the pose into `/hk/adapter/odom` in `map`, just like the simulator.
- With **Lighthouse**, the `map` origin is the origin of the space calibrated by the base stations.
- With only the **Flow Deck**, the pose is relative to the takeoff point: the `map` origin is where the drone takes off.

## How to verify it

With `simulation_cf.launch.py` running:

```bash
ros2 run tf2_ros tf2_echo map base_link        # where the drone is in the room
ros2 run tf2_ros tf2_echo base_link left_range # where a sensor is on the drone
ros2 run tf2_tools view_frames                 # generates frames.pdf with the full tree
```

## Status and pending work

- [x] Conventions, tree and offsets documented.
- [x] Static TFs of the 6 sensors published by the launch.
- [x] `map -> base_link` published by `state_monitor`.
- [ ] Measure the real offsets on the event drone.
- [ ] Verify the signs with the real drone: moving the drone forward must increase x, and an obstacle on the left must appear in `left_range` (event runbook, step 6).
