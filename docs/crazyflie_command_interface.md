# Crazyflie Command Interface

This document defines the Crazyswarm2 command hierarchy used by the Mobile Robotics Hackathon project.

Its purpose is to isolate the project navigation stack from the ROS interfaces exposed by Crazyswarm2. Only `cf_adapter` should depend directly on the interfaces documented here.

---

## 1. Scope

**TP-08 review completed (2026-10-07), scoped to the development baseline.** Endpoint discovery, callback/physics tests, and firmware source inspection are distinguished below. Hardware flight, ROS transport end-to-end behavior, and event firmware configuration are not validated by these tests. TP-09 implementation remains separate work.

This document records the Crazyflie command interfaces verified on the current development baseline:

| Component | Value |
|---|---|
| ROS 2 | Humble |
| Crazyswarm2 / Crazyflie ROS packages | 1.0.7 |
| Firmware source inspected locally | `16c5630707e92830dcd16455a57b5560e5838735` (`2023.07-340-g16c56307`); not evidence of event firmware version |
| SIL firmware bindings | Local `cffirmware` build; the previously stated `2025.02` version was not established by this review |
| Backend inspected | `sim` |
| Robot namespace | `/cf231` |

The interfaces were inspected using:

```bash
ros2 topic list
ros2 service list
ros2 topic info <topic> --verbose
ros2 service type <service>
ros2 interface show <interface>
```

---

## 2. Command Architecture

The Crazyflie firmware performs the low-level flight control onboard.

The host computer does not calculate motor PWM commands. The host sends high-level commands or streaming setpoints, while the onboard Crazyflie controllers stabilize and control the vehicle.

Two command families are relevant:

1. High-level commander services.
2. Streaming setpoint topics.

```text
Mission / Navigation
        |
        v
    cf_adapter
        |
        +-- High-level commands
        |      takeoff
        |      land
        |      go_to
        |      start_trajectory
        |
        +-- Streaming setpoints
               cmd_full_state
               cmd_hover
               cmd_position
               cmd_vel_legacy
        |
        v
Crazyflie onboard controller
        |
        v
Motors
```

The project navigation stack must not depend directly on `/cf231/*`. Crazyswarm2-specific details remain inside `cf_adapter`.

---

## 3. Identified High-Level Services

The following high-level services were visible during the `sim` backend session.

### 3.1 Takeoff

Observed endpoints:

```text
/cf231/takeoff
/all/takeoff
```

Type:

```text
crazyflie_interfaces/srv/Takeoff
```

Interface:

```text
uint8 group_mask
float32 height
builtin_interfaces/Duration duration
---
```

Purpose:

Move the vehicle vertically to the requested height over the requested duration.

---

### 3.2 Land

Observed endpoints:

```text
/cf231/land
/all/land
```

Type:

```text
crazyflie_interfaces/srv/Land
```

Interface:

```text
uint8 group_mask
float32 height
builtin_interfaces/Duration duration
---
```

Purpose:

Command a landing trajectory toward the requested final height.

---

### 3.3 GoTo

Observed endpoints:

```text
/cf231/go_to
/all/go_to
```

Type:

```text
crazyflie_interfaces/srv/GoTo
```

Interface:

```text
uint8 group_mask
bool relative
geometry_msgs/Point goal
float32 yaw
builtin_interfaces/Duration duration
---
```

The interface uses:

- `goal`: position target.
- `relative`: whether the target is interpreted relative to the current state.
- `yaw`: yaw command.
- `duration`: requested motion duration.

For this project, `go_to` is useful for high-level waypoint execution and diagnostics, but reactive local navigation must not depend exclusively on it.

---

### 3.4 Trajectory Commands

Observed endpoints:

```text
/cf231/upload_trajectory
/cf231/start_trajectory
/all/start_trajectory
```

These services are part of the high-level trajectory execution interface.

Trajectory upload/execution is not required for the initial MVP, but the interface remains available for future use.

---

### 3.5 Emergency

Observed endpoints:

```text
/cf231/emergency
/all/emergency
```

Emergency commands are reserved for safety intervention and must not be treated as a normal landing mechanism.

In the installed SIL, this callback only logs `emergency not yet implemented`; it does not provide an emergency-stop effect.

---

### 3.6 Arm

Observed global endpoint:

```text
/all/arm
```

This service is available in the inspected graph and may be required depending on the hardware/backend configuration used at the event.

---

## 4. Identified Streaming Topics

### 4.1 Full State

Observed topic:

```text
/cf231/cmd_full_state
```

Type:

```text
crazyflie_interfaces/msg/FullState
```

Observed runtime state:

```text
Publisher count: 0
Subscription count: 1
Subscriber: /crazyflie_server
```

Interface:

```text
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 acc
```

The message contains:

- position;
- orientation;
- linear velocity;
- angular velocity;
- acceleration.

This interface is the most complete streaming setpoint available in the inspected backend.

The callback/physics test below exercised this command. `CrazyflieSIL.cmdFullState` sets absolute position modes in XYZ: velocity is part of the full reference, not an independent velocity-only mode. Project HOVER and VELOCITY modes must not be assumed equivalent to copying their fields into FullState.

---

### 4.2 Hover

Observed topic:

```text
/cf231/cmd_hover
```

Type:

```text
crazyflie_interfaces/msg/Hover
```

Observed runtime state:

```text
Publisher count: 0
Subscription count: 1
Subscriber: /crazyflie_server
```

Interface:

```text
std_msgs/Header header
float32 vx
float32 vy
float32 yaw_rate
float32 z_distance
```

Interpretation:

- `vx`: horizontal velocity command in X;
- `vy`: horizontal velocity command in Y;
- `yaw_rate`: yaw-rate command;
- `z_distance`: commanded hover height.

The existence of a topic in the ROS graph confirms that the server subscribes to it, but does not by itself prove identical behavior across all backends. Functional behavior must be validated on the backend used at the event.

The installed SIL callback logs `cmd_hover not yet implemented`; exclude it from functional navigation in this simulation baseline.

---

### 4.3 Position

The message type exists:

```text
crazyflie_interfaces/msg/Position
```

Interface:

```text
std_msgs/Header header
float32 x
float32 y
float32 z
float32 yaw
```

However, during the inspected `sim` session:

```text
Unknown topic '/cf231/cmd_position'
```

Therefore `/cf231/cmd_position` must not be assumed to exist in the current simulation baseline.

Any use of position streaming must be validated explicitly on the backend selected for the event.

---

### 4.4 Legacy Velocity / Attitude Command

Observed topic:

```text
/cf231/cmd_vel_legacy
```

Type:

```text
geometry_msgs/msg/Twist
```

Observed runtime state:

```text
Endpoint type: SUBSCRIPTION
Subscriber: /crazyflie_server
Reliability: RELIABLE
Durability: VOLATILE
Liveliness: AUTOMATIC
```

Despite the topic name, this legacy interface must not be interpreted automatically as a conventional world-frame velocity command.

It should not be used by the navigation stack baseline unless its semantics are verified explicitly.

---

## 5. Returning from Streaming Control to High-Level Control

Observed service:

```text
/cf231/notify_setpoints_stop
```

Type:

```text
crazyflie_interfaces/srv/NotifySetpointsStop
```

Interface:

```text
uint8 group_mask
uint32 remain_valid_millisecs
---
```

This service is used when streaming setpoints are being stopped.

Intended hardware sequence, subject to backend validation:

```text
Streaming navigation
        |
        v
notify_setpoints_stop
        |
        v
High-level command
        |
        +--> go_to
        +--> land
        +--> trajectory
```

The request contains `remain_valid_millisecs`, but its effect is firmware-dependent. In the inspected local firmware, `crtp_commander.c::notifySetpointsStopDecoder` ignores this historical argument and calls `commanderRelaxPriority()` immediately. It is not a guaranteed delay or validity period.

Stop generating streaming commands before requesting the transfer; otherwise new streaming commands can take priority again. Check service errors before continuing. In the installed SIL, `notify_setpoints_stop` is unimplemented. The tested streaming-to-go_to transition raises `goTo from low-level modes not yet supported.` Landing after FullState was exercised successfully below; this does not prove an equivalent hardware transition.

---

## 6. Command Priority and Watchdog Considerations

The project must treat high-level commands and streaming setpoints as different command modes.

A streaming command path requires periodic command updates from the host. If that stream becomes stale, the safety layer must not continue assuming normal navigation.

The local firmware source establishes the following behavior, not a measurement on event hardware:

- `commander.h` assigns high-level priority 1 and CRTP streaming priority 2. `commanderSetSetpoint` accepts equal or higher priority; higher-than-high-level commands stop the onboard high-level planner. `commanderRelaxPriority` releases priority and supplies the last state to the high-level commander.
- `supervisor.c` checks setpoint age: more than 500 ms raises a warning; more than 2000 ms raises a timeout.
- In the flying state, warning selects `WarningLevelOut`: XY position modes are disabled, roll/pitch references are zeroed, yaw rate is zeroed, and Z is retained. Timeout selects an exception state; the supervisor override supplies a null setpoint to stop motors. This is not a normal automatic landing.
- SIL does not execute that full commander/supervisor watchdog path. Withholding FullState updates for 3 simulated seconds retained the last target and continued control in the test.

Source files inspected: `commander.c`, `commander.h`, `crtp_commander.c`, `supervisor.c` and `supervisor_state_machine.c` in the local firmware checkout at the revision above. Confirm actual event firmware, configuration and policy with mentors. Host timeouts remain configurable; do not rely on SIL to reproduce hardware failsafes.

Project policy:

- do not hardcode firmware watchdog values in navigation logic;
- expose host-side command timeout values through configuration;
- let `safety_supervisor` detect stale command generation;
- use `notify_setpoints_stop` before returning from streaming control to high-level commands;
- use `land` for normal landing;
- reserve `emergency` for safety-critical intervention.

---

## 7. Previously Observed ROS Graph

This inventory records the earlier session, not functional support or a guarantee that every endpoint is created by the current SIL server. `/all/arm` was recorded previously but is not created by the inspected SIL server code.

During the inspected `sim` session, the following services were visible:

```text
/all/arm
/all/emergency
/all/go_to
/all/land
/all/start_trajectory
/all/takeoff

/cf231/emergency
/cf231/go_to
/cf231/land
/cf231/notify_setpoints_stop
/cf231/start_trajectory
/cf231/takeoff
/cf231/upload_trajectory
```

The following Crazyflie command topics were visible:

```text
/cf231/cmd_full_state
/cf231/cmd_hover
/cf231/cmd_vel_legacy
/cf231/robot_description
```

The following topic was explicitly not present:

```text
/cf231/cmd_position
```

---

## 8. Available Crazyflie Interfaces

These are installed type definitions, not active endpoints. A topic name identifies a channel; its `msg` type defines each message. A service name identifies a request/response endpoint; its `srv` type defines both structures. An installed type can exist without any running endpoint using it.

The installed `crazyflie_interfaces` package exposes the following relevant command interfaces.

### Messages

```text
crazyflie_interfaces/msg/FullState
crazyflie_interfaces/msg/Hover
crazyflie_interfaces/msg/Position
crazyflie_interfaces/msg/TrajectoryPolynomialPiece
crazyflie_interfaces/msg/VelocityWorld
```

### Services

```text
crazyflie_interfaces/srv/Arm
crazyflie_interfaces/srv/GoTo
crazyflie_interfaces/srv/Land
crazyflie_interfaces/srv/NotifySetpointsStop
crazyflie_interfaces/srv/StartTrajectory
crazyflie_interfaces/srv/Stop
crazyflie_interfaces/srv/Takeoff
crazyflie_interfaces/srv/UploadTrajectory
```

Interface availability does not imply that every backend exposes or implements every command endpoint.

---

## 9. Project Abstraction

The rest of the project must use the internal Hackathon API instead of Crazyswarm2 endpoints directly.

Expected internal command flow:

```text
mission_manager
      |
      v
local_navigator
      |
      v
/hk/cmd/nav
      |
      v
safety_supervisor
      |
      v
/hk/cmd/safe
      |
      v
cf_adapter
      |
      +--> Crazyswarm2 service/topic
```

Only `cf_adapter` translates between the project API and the Crazyflie ROS API.

Expected project-facing interfaces include:

```text
/hk/state
/hk/ranges
/hk/cmd/nav
/hk/cmd/safe
/hk/takeoff
/hk/land
/hk/emergency
```

This separation allows the same navigation and safety stack to work with:

```text
simulation_adapter
```

or:

```text
cf_adapter + Crazyswarm2
```

without changing planner, mission, perception, or safety logic.

---

## 10. Baseline Decision for the MVP

The current command policy is:

- use high-level `takeoff` for normal takeoff;
- use high-level `land` for normal landing;
- keep `go_to` available for simple waypoint execution and diagnostics;
- do not couple reactive navigation directly to `go_to`;
- use `cmd_full_state` as the streaming candidate exercised in the SIL callback/physics test, with absolute-position semantics and the limitations below;
- exclude `cmd_hover` from functional SIL navigation because its callback is unimplemented; validate support on other backends;
- do not assume `/cf231/cmd_position` exists in the current simulation backend;
- exclude `cmd_vel_legacy` from the navigation baseline unless its exact semantics are explicitly required;
- validate `notify_setpoints_stop` and handover on hardware; SIL does not implement priority release, and streaming-to-go_to is unsupported in the tested sequence;
- route all Crazyflie-specific commands through `cf_adapter`;
- let `safety_supervisor` own the final command path before commands reach an adapter.

---

## 11. TP-08 Definition of Done

TP-08 is complete when:

- takeoff is documented;
- land is documented;
- go-to is documented;
- trajectory services are identified;
- streaming command interfaces are documented;
- the difference between high-level and streaming control is clear;
- `notify_setpoints_stop` is documented;
- host-side stale-command handling is assigned to `safety_supervisor`;
- the low-level flight controller is confirmed to remain onboard;
- Crazyflie-specific ROS endpoints are isolated behind `cf_adapter`.

The development-baseline documentation review satisfies these criteria. Event-specific hardware verification remains in the integration runbook. This does not imply TP-09 is complete.

## 12. Reproducible SIL verification (2026-10-07)

`scripts/verify_tp08.py` creates the installed Crazyswarm2 simulation server with `np` physics, the `mellinger` controller and only simulated `cf231`. It calls actual command callbacks directly and advances physics in 0.0005 s steps. No radio or hardware is used. This does not test ROS request transport, RViz, range sensing, obstacle navigation, or `cf_adapter`.

Run from the repository root in this Ubuntu environment:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="/home/asus/crazyflie-firmware/build${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOG_DIR=/tmp/tp08-ros-logs
/usr/bin/python3 scripts/verify_tp08.py
```

To watch the same sequence, replace the last command with:

```bash
/usr/bin/python3 scripts/verify_tp08.py --visual
```

Visual mode publishes the simulated transform and robot model, opens RViz with simulation time, and labels each command in the terminal. It limits TF publication to 50 Hz and prevents advancing faster than real time; CPU load can make it slower. Press Enter at the end to close RViz, or Ctrl+C to stop. `--visual --no-rviz` publishes visualization data for an external RViz in ROS domain 88. This is the Crazyswarm2 SIL test, not the project's obstacle simulator or navigation stack.

The script isolates ROS in domain 88 on localhost and shuts down its simulation on exit. The bindings path is machine-specific; colleagues should use their own build path. Test values are simulation inputs, not approved hardware settings.

| Check | Result |
|---|---|
| Takeoff to 0.5 m over 2 s; observe after 3 s | PASS: Z ≈ 0.428 m |
| Absolute go_to to `(0.3, 0, 0.5)` over 2 s | PASS: X ≈ 0.299 m, Z ≈ 0.433 m |
| FullState `(0.4, 0, 0.5)` at 50 Hz for 2 simulated seconds | PASS: X ≈ 0.409 m, Z ≈ 0.433 m |
| Stop FullState for 3 simulated seconds | Last target retained: X ≈ 0.404 m, Z ≈ 0.433 m; no automatic landing |
| notify_setpoints_stop, cmd_hover, emergency | Logs report unimplemented; command mode unchanged |
| go_to after FullState and notify_setpoints_stop | Expected rejection: low-level transition unsupported |
| Land to Z=0 over 2 s after FullState | PASS: reaches simulated ground after 3 s |

Position acceptance tolerance is 0.12 m (0.15 m for landing). The persistent altitude offset of about 0.067 m is visible; these tests establish basic effects, not precise tracking or a tuned flight model. Trajectory upload/execution was identified but not exercised because it is outside the MVP.
