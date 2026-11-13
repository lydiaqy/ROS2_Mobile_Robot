# ROS 2 Autonomous Mobile Robot

A differential-drive mobile robot simulation project built with **ROS 2 Jazzy, C++, and Gazebo Harmonic**.

The project focuses on robot motion control, odometry feedback, simulation validation, and autonomous navigation.

## Features

- Differential-drive robot modeled using URDF/Xacro
- Gazebo physics simulation
- ROS 2 C++ publisher/subscriber nodes
- Odometry-based distance and rotation control
- Finite State Machine (FSM) for square-path execution
- Acceleration-limited drive commands and odometry-based settling between motions
- Differential-drive parameter calibration

## Tech Stack

- C++17
- ROS 2 Jazzy
- Gazebo Harmonic
- URDF / Xacro
- ros_gz_bridge
- Ubuntu 24.04 / WSL2

## Current Results

The robot can execute a four-sided square trajectory using odometry feedback and proportional control.

In one test with a target side length of 1.0 m, the controller reported a final odometry-based position closure error of approximately 0.05 m.

The difference between wheel odometry and Gazebo ground-truth rotation was also investigated through empirical differential-drive calibration.

The current drive configuration keeps the physical wheel separation at 0.34 m
and uses an empirically calibrated effective separation of 0.304 m in DiffDrive,
with linear acceleration limited to ±0.3 m/s² and angular acceleration to
±0.8 rad/s². The square-path controller commands zero velocity between motions
and waits for odometry speeds to remain below 0.01 m/s and 0.02 rad/s for 0.5 s.
These thresholds are configurable through `stop_linear_tolerance`,
`stop_angular_tolerance`, and `settle_time`. This checks wheel-odometry motion;
it does not measure body pitch or ground-truth stability.

The measurements above predate these changes and should be revalidated for the
current configuration.

See [Calibration Analysis](docs/calibration.md) for experimental measurements, limitations, and further investigation.

## Build

```bash
cd ~/mobile_robot_ws
colcon build --symlink-install \
  --packages-select mobile_robot_control mobile_robot_description mobile_robot_simulation
source install/setup.bash
```

## Run Simulation

```bash
ros2 launch mobile_robot_description spawn_robot.launch.py
```

In a separate terminal:

```bash
source ~/mobile_robot_ws/install/setup.bash

ros2 run mobile_robot_control square_path_controller \
  --ros-args \
  -p side_length:=1.0 \
  -p max_linear_speed:=0.2 \
  -p max_angular_speed:=0.4
```

## Roadmap

- [x] Differential-drive simulation
- [x] ROS 2 C++ motion controllers
- [x] Square-path FSM controller
- [x] Initial odometry calibration
- [ ] 2D LiDAR integration
- [ ] Obstacle detection and avoidance
- [ ] SLAM and autonomous navigation

## Project Status

**V0.1 — Motion Control Prototype**

This is an actively developed learning and portfolio project. The current simulation and controller behavior have been tested in the development environment; additional validation and automated tests are planned.
