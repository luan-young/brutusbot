# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Brutusbot is a ROS 2 **Jazzy** differential-drive robot (Raspberry Pi 5 running ROS 2 + `ros2_control`, and an Arduino driving two stepper motors). This repo is a set of ament_cmake packages living in `~/ros2_brutusbot_ws/src/brutusbot`. The colcon workspace root is `~/ros2_brutusbot_ws`.

## Build / run

Run from the workspace root (`~/ros2_brutusbot_ws`), not from this repo:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install                          # all packages
colcon build --packages-select brutusbot_firmware        # single package
source install/setup.bash

ros2 launch brutusbot_bringup simulated_robot.launch.py  # Gazebo + controllers + joystick teleop
ros2 launch brutusbot_bringup real_robot.launch.py       # real hardware (Arduino on /dev/ttyUSB0)
ros2 launch brutusbot_description display.launch.py      # URDF in RViz with joint_state_publisher_gui
```

Tests: there are no unit tests; `colcon test` only runs `ament_lint_auto` (copyright and cpplint are disabled in every package). `brutusbot_firmware` builds two manual hardware-check executables that talk to the Arduino on `/dev/ttyUSB0`:

```bash
ros2 run brutusbot_firmware test_serial_protocol   # exercises the framed protocol (velocity cmd + state request)
ros2 run brutusbot_firmware test_serial            # raw serial echo check
```

C++ formatting: `.clang-format` (Google style, 4-space indent, `AccessModifierOffset: -4`, 120 column limit).

`brutusbot_firmware/build/` is a stray local CMake build dir (has `AMENT_IGNORE`, gitignored) — ignore it.

## Raspberry Pi 5 access

SSH credentials for the robot's Pi 5 live in `.secrets/pi5.env` (gitignored, `chmod 600`; never commit it or echo the password into output). It defines `PI5_HOST`, `PI5_USER`, `PI5_PASSWORD`. To run a command on the Pi:

```bash
source .secrets/pi5.env
sshpass -p "$PI5_PASSWORD" ssh -o StrictHostKeyChecking=accept-new "$PI5_USER@$PI5_HOST" '<command>'
```

`sshpass` must be installed (`sudo apt install sshpass`); if an SSH key is set up for the Pi, plain `ssh "$PI5_USER@$PI5_HOST"` works and the password isn't needed.

## Architecture

Package dependency flow: `brutusbot_bringup` → (`brutusbot_description`, `brutusbot_controller`, `brutusbot_firmware`).

- **brutusbot_description** — Xacro model. `urdf/brutusbot.xacro` takes an `is_sim` xacro arg that switches the whole stack between sim and hardware:
  - `is_sim=true`: includes `brutusbot_gazebo.xacro` (wheel friction + the `gz_ros2_control` Gazebo plugin, which loads `brutusbot_controller/config/brutusbot_controllers.yaml` itself) and `brutusbot_ros2_control.xacro` selects `gz_ros2_control/GazeboSimSystem`.
  - `is_sim=false`: `brutusbot_ros2_control.xacro` selects `brutusbot_firmware/BrutusbotInterface` with a `serial_port` hardware param.
  - `gazebo.launch.py` processes xacro with `is_sim=true`, spawns into `worlds/empty.sdf` (Gazebo's stock empty world + the Sensors system, needed for the camera), bridges `/clock`, `/image_raw`, `/camera_info` and `/ground_truth/pose`, and opens RViz.
- **brutusbot_controller** — config + launch only. `brutusbot_controllers.yaml` defines `controller_manager` (100 Hz), a `diff_drive_controller` named `brutusbot_controller` (wheel joints `left_motor_wheel_joint`/`right_motor_wheel_joint`, separation 0.23 m, radius 0.05 m, base frame `base_footprint`, stamped cmd_vel) and `joint_state_broadcaster`. `controller.launch.py` just spawns both controllers. `joystick_teleop.launch.py` runs `joy_node` + `joy_teleop`, publishing `TwistStamped` to `/brutusbot_controller/cmd_vel` (deadman button 5). Velocity limits in the controller YAML and the scales in `joy_teleop.yaml` are kept consistent (0.75 m/s, 2.17 rad/s) — change them together.
- **brutusbot_firmware** — the real-robot side:
  - `BrutusbotInterface` (`src/brutusbot_interface.cpp`): a `hardware_interface::SystemInterface` pluginlib plugin (registered via `brutusbot_interface.xml`). Converts wheel velocity commands rad/s → deg/s for the Arduino and state replies deg → rad. Uses libserial at 115200 baud; tolerates up to `kMaxReadErrors`/`kMaxWriteErrors` consecutive failures before returning an error; `on_activate` retries the initial state request.
  - `launch/hardware_interface.launch.py`: processes xacro with `is_sim=false` and starts `robot_state_publisher` + `ros2_control_node` (the real-robot equivalent of what Gazebo provides in sim).
  - Serial protocol (`serial_protocol.{hpp,cpp}`): frame = `0x7E` start, payload size, size checksum, payload (message code + data), additive checksum over size+payload; max payload 32 bytes; byte-by-byte state-machine parser. Codes: `0x10` set wheel velocity (2 floats), `0x11` request state, `0x20` velocity ack, `0x21` state reply (left pos/vel, right pos/vel as floats).
  - `arduino/brutusbot_control/`: the Arduino sketch (AccelStepper) with its own copy of the protocol in `serial_protocol.h`. **Any protocol change must be mirrored in both the ROS side and the Arduino header** (and the tables in `arduino/README.md`).
- **brutusbot_bringup** — top-level launch files composing the above. Sim passes `use_sim_time=True`; real robot passes `False`. Note `brutusbot_controllers.yaml` sets `use_sim_time: true` and is reused by the real-robot launch, which overrides it with `use_sim_time: False`.
- **brutusbot_slam** — monocular visual SLAM work in progress on the `visual-slam` branch. The phased plan, status and decision log are in `brutusbot_slam/PLAN.md`. Read it before working on SLAM or Nav2, and update its status table and decision log as phases finish or decisions change.
