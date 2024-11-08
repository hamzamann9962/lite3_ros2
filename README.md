# Lite3 ROS2

This repository is an unofficial project for DeepRobotics Lite3.

## 1. Quick Start

* prepare workspace
```bash
cd ~
mkdir ysc_ws
cd ysc_ws
mkdir src
cd src
```

* Clone the repository
```bash
git clone https://github.com/legubiao/Lite3_ROS2
cd Lite3_ROS2
git submodule update --init --recursive
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

### 1.1 Visualize Robot Joints
```bash
source ~/ysc_ws/install/setup.bash
ros2 launch lite3_description visualize.launch.py
```

### 1.2 Visualize Lidar
Setup and compile the lslidar ROS2 package, then the lidar can be visualized in rviz.
```bash
source ~/ysc_ws/install/setup.bash
ros2 launch lslidar_driver lslidar_cx_rviz_launch.py
```

![lidar](.images/lidar.png)