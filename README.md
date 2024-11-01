# Lite3 ROS2

This repository is an unofficial project for DeepRobotics Lite3.

## Quick Start

* prepare workspace
```bash
cd ~
mkdir ysc_ws
cd ysc_ws
mkdir src
cd src
```
* rosdep
```bash
cd ~/ysc_ws
rosdep install --from-paths src --ignore-src -r -y
```

* compile packages
```bash
cd ~/ysc_ws
colcon build --packages-up-to lite3_udp_bridge
```

* launch
```bash
source ~/ysc_ws/install/setup.bash
ros2 launch lite3_udp_bridge bridge.launch.py
```

* keyboard control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```