# MVP sensors and integration

The API of the person 2 branches is kept:

| cf_adapter input | Type | Convention |
|---|---|---|
| `/cf231/pose` | PoseStamped | pose in `world`, measurement timestamp |
| `/cf231/scan` | LaserScan | `[back, right, front, left]`, metres, angles −π + i·π/2 |
| `/cf231/ranges_extra` | LogDataGeneric | `[up, down]`, millimetres |

`cf_adapter` publishes `/hk/adapter/odom` and `/hk/adapter/ranges`.
`state_monitor` publishes `/hk/state`, `/hk/localization_health` and the TF `map → base_link`.
`range_processor` publishes `/hk/ranges`, `/hk/ranges/closing_rate` and `/hk/ranges/markers`.

## Two ways to simulate

- `backend:=emulator`: the person 2 `sim_crazyflie` generates kinematic dynamics,
  pose, raycasts and faults. Its models live in `simulation_adapter/emulator` to keep
  the original TP-10. It does not run real firmware; its watchdog is an approximation.
- `backend:=sil`: the external Crazyswarm2 server generates the motion and `/clock`.
  `sim_sensors` reads the TF `world → cf231`, adds pose noise and computes rays against
  the scenario world. It takes the full orientation and the sensor offsets into account.
  It generates no motion, EKF or optical flow, and does not implement the emulator faults.

The scan is clipped at 3.49 m. `+inf` means no return, `-inf` too close, `NaN` invalid.
`-inf` passes through the filter immediately; no confirmation or median is awaited.
NaN may hold the last value for `invalid_hold` (0.3 s). After that, an invalid
horizontal ray causes HOVER if `ranges_required` is enabled.
Individual rays model neither the optical cone nor materials.

The rates use the change of the filtered distance: negative values mean approaching.
The navigator holds the yaw (`face_target: false`, the default) so that a turn does not
look like a closing speed. Safety considers TTC and braking (`max_decel` 0.5 m/s² by
default, `reaction_time` 0.3 s); these values need tuning and testing on hardware.

## Moving to hardware

Replace the simulator and `sim_sensors` with the hardware server, and configure logging of
`pose`, `scan` and the custom topic `ranges_extra` with the variables `[range.up, range.zrange]`.
Verify the real order, units, timestamps, frames, invalid return and rate.
Do not change the team topics. See [crazyflie_command_interface.md](crazyflie_command_interface.md):
the emergency and notify services of the installed SIL do not prove functional support on hardware.

The integration validation and the commands for native Ubuntu/ROS are at the end of
[simulations.md](simulations.md).
