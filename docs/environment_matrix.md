# Environment Matrix

This document defines the software baseline used by the team for the Mobile Robotics Hackathon 2026.

The purpose of this matrix is to keep all development machines aligned and to make the project reproducible across team members.

The baseline should only be changed intentionally and documented through a pull request.

## Current Development Baseline

| Component | Version | Status | Verification command | Notes |
|---|---:|---|---|---|
| Operating System | Ubuntu 22.04.5 LTS (Jammy Jellyfish) | ✅ Verified | `cat /etc/os-release` | Development environment under WSL 2 |
| Architecture | x86_64 | ✅ Verified | `uname -m` | Team development baseline |
| Shell | Bash | ✅ Verified | `echo $SHELL` | `/bin/bash` |
| Python | 3.10.12 | ✅ Verified | `python3 --version` | System Python used by ROS 2 Humble |
| GCC | 11.4.0 | ✅ Verified | `gcc --version` | Ubuntu Jammy toolchain |
| CMake | 3.22.1 | ✅ Verified | `cmake --version` | Used for ROS 2 / CMake packages |
| ROS 2 | Humble | ✅ Verified | `echo $ROS_DISTRO` | Main ROS 2 distribution for the project |
| ROS 2 CLI | Humble | ✅ Verified | `ros2 --help` | CLI operational |
| colcon-core | 0.21.3 | ✅ Verified | `colcon version-check` | ROS 2 workspace build tool |
| rosbag2 | Humble package installed | ✅ Verified | `ros2 bag --help` | Runtime availability verified |
| Git | TBD | 🟡 Pending version capture | `git --version` | Repository version control |
| VS Code | 1.106.3 | ✅ Verified | `code --version` | Running through VS Code Remote WSL |
| RViz2 | ROS 2 Humble package | ✅ Verified | `rviz2 --help` | Successfully launched through Crazyswarm2 under WSLg |
| Crazyswarm2 / Crazyflie ROS packages | 1.0.7 | ✅ Verified | `dpkg -l \| grep ros-humble-crazyflie` | Binary ROS 2 Humble packages |
| Crazyflie firmware | 2025.02 | ✅ Verified | `git describe --tags --always` | Firmware source used for SIL Python bindings |
| cffirmware | Built from firmware 2025.02 | ✅ Verified | `PYTHONPATH=build python3 -c "import cffirmware"` | Native Python binding compiled successfully; 22/22 firmware tests passed |
| rowan | 1.3.2 | ✅ Verified | `python3 -c "import rowan; print(rowan.__version__)"` | Required by the Crazyswarm2 simulation backend |
| SWIG | 4.0.2 | ✅ Verified | `swig -version` | Required to generate `cffirmware` Python bindings |
| Crazyradio | TBD | ⬜ Event hardware | TBD | USB/radio access must be validated separately on event hardware |

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
echo $SHELL

python3 --version
gcc --version | head -n 1
cmake --version | head -n 1

echo $ROS_DISTRO
ros2 --help >/dev/null && echo "ROS 2 CLI OK"
colcon version-check
ros2 bag --help >/dev/null && echo "rosbag2 OK"

git --version
code --version

dpkg -l | grep ros-humble-crazyflie
python3 -c "import rowan; print(rowan.__version__)"
swig -version