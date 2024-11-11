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
colcon build --packages-up-to lite3_udp_bridge --symlink-install
```

* launch

```bash
source ~/ysc_ws/install/setup.bash
ros2 launch lite3_udp_bridge bridge.launch.py
```

### 1.1 Visualize Robot

* To visualize and check the configuration of the robot in rviz, simply launch:

  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description visualize.launch.py check_gui:=true
  ```

![joints](.images/joints.png)

* To visualize the robot with the actual joint states data, launch:
  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description visualize.launch.py
  ```

![odom](.images/odom.png)

* To visualize the robot with lidar data, launch:
  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description lslidar.launch.py
  ```

![lidar](.images/lidar.png)

## 2. SLAM

### 2.1 SLAM Toolbox
* Install
  ```bash
  sudo apt-get install ros-jazzy-slam-toolbox
  ```
* Launch
  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description slam_toolbox.launch.py
  ```
  
### 2.1 Cartographer

* Install
  ```bash
  sudo apt-get install ros-jazzy-cartographer-ros
  ```
* Launch
  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description cartographer.launch.py
  ```

### 2.2 Fast-LIO

  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description fast_lio.launch.py
  ```

![fastlio](.images/fast_lio.png)