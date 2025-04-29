# Raspberry Pi Repository - SLAM Vehicle Setup

This repository contains the essential nodes and launch files for running the Raspberry Pi side of the SLAM-based autonomous vehicle.

---

## Overview

The Raspberry Pi is responsible for:
- Publishing real-time LiDAR scan data to the `/scan` topic.
- Controlling the motors using a custom Ackermann steering controller.

Both nodes must be launched separately and continuously for the full system operation.

---

## Requirements

- ROS 2 Humble/Foxy installed on Raspberry Pi
- RPLIDAR C1 sensor and drivers (`rplidar_ros` package)
- Custom Ackermann motor control package (`ackermann_control`)
- Proper environment sourced:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Launch Instructions

### 1. Launch the LiDAR Node

This node will publish real-time laser scans to the `/scan` topic.

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```

- Make sure the RPLIDAR is properly connected via USB.

### 2. Launch the Ackermann Controller Node

This node accepts velocity commands and converts them into motor actuation.

```bash
ros2 run ackermann_control ackermann_motor_controller
```

- It listens to the `/cmd_vel` topic and sends PWM signals to the motors.

> **Note**: Launch each of the above nodes in separate terminal sessions.

---

## Recommended Workflow

1. SSH into Raspberry Pi or open two terminal windows.
2. In Terminal 1, launch the LiDAR node.
3. In Terminal 2, launch the Ackermann motor controller.
4. Ensure WiFi connectivity between Raspberry Pi and Host Machine for ROS 2 communication.

---

## Troubleshooting

- **No LiDAR data**: Check USB permissions and device connection.
- **No motor movement**: Confirm motor controller wiring and power supply.
- **ROS 2 node not found**: Make sure the workspace is built and sourced properly.

```bash
colcon build --symlink-install
source install/setup.bash
```

---
