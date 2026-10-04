# Crazyflie Command Interface

This document defines the Crazyswarm2 command hierarchy used by the Mobile Robotics Hackathon project.

Its purpose is to isolate the project navigation stack from the ROS interfaces exposed by Crazyswarm2. Only `cf_adapter` should depend directly on the interfaces documented here.

---

## 1. Scope

This document records the Crazyflie command interfaces verified on the current development baseline:

| Component | Value |
|---|---|
| ROS 2 | Humble |
| Crazyswarm2 / Crazyflie ROS packages | 1.0.7 |
| Crazyflie firmware used by SIL | 2025.02 |
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

## 3. Verified High-Level Services

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

---

### 3.6 Arm

Observed global endpoint:

```text
/all/arm
```

This service is available in the inspected graph and may be required depending on the hardware/backend configuration used at the event.

---

## 4. Verified Streaming Topics

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

Conceptually:

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

The parameter `remain_valid_millisecs` defines how long the previous streaming setpoint should remain valid during the transition.

The project must use this transition mechanism when handing control back from streaming setpoints to high-level commands.

---

## 6. Command Priority and Watchdog Considerations

The project must treat high-level commands and streaming setpoints as different command modes.

A streaming command path requires periodic command updates from the host. If that stream becomes stale, the safety layer must not continue assuming normal navigation.

The exact firmware watchdog behavior and timeout values must be validated against the firmware and backend used at the event before hardcoding any safety timing.

Project policy:

- do not hardcode firmware watchdog values in navigation logic;
- expose host-side command timeout values through configuration;
- let `safety_supervisor` detect stale command generation;
- use `notify_setpoints_stop` before returning from streaming control to high-level commands;
- use `land` for normal landing;
- reserve `emergency` for safety-critical intervention.

---

## 7. Verified ROS Graph

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
- keep `cmd_full_state` as the primary fully verified streaming candidate in the current `sim` baseline;
- treat `cmd_hover` as available in the inspected ROS graph, but validate behavior before depending on it;
- do not assume `/cf231/cmd_position` exists in the current simulation backend;
- exclude `cmd_vel_legacy` from the navigation baseline unless its exact semantics are explicitly required;
- use `notify_setpoints_stop` when transitioning from streaming setpoints back to high-level commands;
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

