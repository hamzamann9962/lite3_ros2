# DeepRobotics Lite3 Description

This repository contains the urdf model of lite3.

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy

## Build

```bash
cd ~/ysc_ws
colcon build --packages-up-to lite3_description --symlink-install
```

## Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/ysc_ws/install/setup.bash
ros2 launch lite3_description visualize.launch.py
```