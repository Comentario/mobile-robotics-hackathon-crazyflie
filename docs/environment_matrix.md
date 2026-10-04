# Environment Matrix

This document defines the software baseline used by the team for the Mobile Robotics Hackathon 2026.

The purpose of this matrix is to keep all development machines aligned and to make the project reproducible across team members.

The baseline should only be changed intentionally and documented through a pull request.

## Current Development Baseline

| Component                       |                              Version | Status                     | Verification command   | Notes                                        |
| ------------------------------- | -----------------------------------: | -------------------------- | ---------------------- | -------------------------------------------- |
| Operating System                | Ubuntu 22.04.5 LTS (Jammy Jellyfish) | ✅ Verified                 | `cat /etc/os-release`  | Development environment under WSL 2          |
| Architecture                    |                               x86_64 | ✅ Verified                 | `uname -m`             | Team development baseline                    |
| Shell                           |                                 Bash | ✅ Verified                 | `echo $SHELL`          | `/bin/bash`                                  |
| Python                          |                              3.10.12 | ✅ Verified                 | `python3 --version`    | System Python used by ROS 2 Humble           |
| GCC                             |                               11.4.0 | ✅ Verified                 | `gcc --version`        | Ubuntu Jammy toolchain                       |
| CMake                           |                               3.22.1 | ✅ Verified                 | `cmake --version`      | Used for ROS 2 / CMake packages              |
| ROS 2                           |                               Humble | ✅ Verified                 | `echo $ROS_DISTRO`     | Main ROS 2 distribution for the project      |
| ROS 2 CLI                       |                               Humble | ✅ Verified                 | `ros2 --help`          | CLI operational                              |
| colcon-core                     |                               0.21.3 | ✅ Verified                 | `colcon version-check` | ROS 2 workspace build tool                   |
| rosbag2                         |             Humble package installed | ✅ Verified                 | `ros2 bag --help`      | Exact package version can be added later     |
| Git                             |                                  TBD | 🟡 Pending version capture | `git --version`        | Repository version control                   |
| VS Code                         |                                  TBD | 🟡 Pending version capture | `code --version`       | Used through VS Code Remote WSL              |
| RViz2                           |                                  TBD | 🟡 Pending validation      | `rviz2 --help`         | Visualization tool                           |
| Crazyswarm2                     |                                  TBD | ⬜ Pending                  | TBD                    | Crazyflie ROS 2 integration                  |
| Crazyflie firmware / cffirmware |                                  TBD | ⬜ Pending                  | TBD                    | Required by the Crazyflie stack/simulation   |
| rowan                           |                                  TBD | ⬜ Pending                  | TBD                    | Python dependency used by the simulation     |
| Crazyradio                      |                                  TBD | ⬜ Event hardware           | TBD                    | Hardware access must be validated separately |

## Baseline Policy

The currently accepted development baseline is:

**Ubuntu 22.04 LTS + ROS 2 Humble + Python 3.10**

All team members should reproduce this baseline before working on ROS 2 packages.

New tools and dependencies should be added to this matrix when they become part of the project baseline. Important version changes should be introduced through a pull request so that the rest of the team can reproduce them.

## Environment Validation

The following commands provide a quick environment check:

```bash
cat /etc/os-release
uname -m
python3 --version
gcc --version | head -n 1
cmake --version | head -n 1
echo $ROS_DISTRO
ros2 --help >/dev/null && echo "ROS 2 CLI OK"
colcon version-check
ros2 bag --help >/dev/null && echo "rosbag2 OK"
git --version
```

Expected core baseline:

```text
Ubuntu: 22.04 LTS
Architecture: x86_64
Python: 3.10.x
GCC: 11.x
CMake: 3.22.x
ROS_DISTRO: humble
```

## Future Entries

The following components will be added or updated as the corresponding roadmap TPs are completed:

* Crazyswarm2
* Crazyflie ROS packages
* Crazyflie firmware / cffirmware
* RViz2
* rowan
* Crazyradio / USB configuration
* Simulation dependencies
* Additional ROS 2 packages required by the navigation stack
