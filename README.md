# Lite3_ROS2

This repository is an unofficial ROS 2 interface for the DeepRobotics Lite3 quadruped robot. It provides packages for UDP communication, robot visualization, LiDAR integration, and SLAM. This guide helps you install and run the project on Ubuntu 24.04 with ROS 2 Jazzy, including RViz visualization for users without a physical robot.

## Prerequisites

- **Operating System**: Ubuntu 24.04 (Noble).
- **ROS 2 Version**: Jazzy Jalisco.
- **Hardware**: Optional DeepRobotics Lite3 robot. RViz visualization works without hardware.
- **Tools**: `git`, `colcon`, `rosdep`, `rviz2`.

### Install Prerequisites

1. **Install ROS 2 Jazzy**:

   ```bash
   sudo apt update
   sudo apt install ros-jazzy-desktop
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Install Build Tools**:

   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

3. **Initialize rosdep**:

   ```bash
   sudo rosdep init
   rosdep update
   ```

## 1. Quick Start

This section guides you through setting up the workspace, cloning the repository, installing dependencies, building, and launching the communication bridge.

### 1.1 Prepare Workspace

Create a ROS 2 workspace (e.g., `lite3_ws`). Avoid using an existing workspace to prevent conflicts.

```bash
cd ~
mkdir -p lite3_ws/src
cd lite3_ws/src
```

### 1.2 Clone the Repository

Clone the Lite3_ROS2 repository and its submodules.

```bash
git clone https://github.com/<your_username>/Lite3_ROS2
cd Lite3_ROS2
git submodule update --init --recursive
```

### 1.3 Install Dependencies

Install system and ROS 2 dependencies, including `rosbridge_server` for the bridge.

```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite
cd ~/lite3_ws
rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:noble
```

**Note**: If `ros-jazzy-rosbridge-suite` is unavailable, build from source:

```bash
cd ~/lite3_ws/src
git clone --depth 1 -b jazzy https://github.com/RobotWebTools/rosbridge_suite
cd ~/lite3_ws
rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:noble
colcon build --packages-up-to rosbridge_server --symlink-install
```

### 1.4 Compile Packages

Build the core packages for communication and visualization.

```bash
cd ~/lite3_ws
colcon build --packages-up-to lite3_udp_bridge lite3_description --symlink-install
```

### 1.5 Launch the Bridge

Launch the UDP bridge to connect ROS 2 with the Lite3 robot’s QNX system.

```bash
source ~/lite3_ws/install/setup.bash
ros2 launch lite3_description bridge.launch.py
```

**Output**: Expect nodes `qnx2ros`, `ros2qnx`, and `rosbridge_websocket` to start, with a UDP server on port 43897 and WebSocket on port 9090.

**Note**: Without a physical robot, the bridge runs but may not publish dynamic topics (e.g., `/joint_states`). Use RViz for visualization (see below).

## 2. RViz Visualization

Visualize the Lite3 robot model in RViz, which works without a physical robot.

### 2.1 Basic Visualization

Visualize the robot’s static model with a GUI to adjust joint angles.

```bash
source ~/lite3_ws/install/setup.bash
ros2 launch lite3_description visualize.launch.py check_gui:=true
```

- **Output**: RViz opens, displaying the Lite3 model. Use the `joint_state_publisher_gui` to manually adjust joint angles, simulating movement.
- **RViz Setup**: Add `RobotModel` (`/robot_description`) and `TF` displays.

### 2.2 Joint States Visualization

Visualize with joint states (requires the bridge or manual publishing).

```bash
source ~/lite3_ws/install/setup.bash
ros2 launch lite3_description visualize.launch.py
```

**Note**: Without a robot, joint states may be static unless you publish `/joint_states` (see Section 2.3).

### 2.3 Manual Joint State Publishing

Simulate movement in RViz by publishing `/joint_states` manually, useful without a physical robot.

1. Launch RViz:

   ```bash
   source ~/lite3_ws/install/setup.bash
   ros2 launch lite3_description visualize.launch.py check_gui:=true
   ```

2. Find joint names in the URDF:

   ```bash
   ros2 topic echo /robot_description
   ```

3. Publish joint states (replace `joint1`, `joint2` with actual joint names):

   ```bash
   ros2 topic pub /joint_states sensor_msgs/msg/JointState "header: {frame_id: ''}, name: ['joint1', 'joint2'], position: [0.5, -0.5], velocity: [], effort: []" --once
   ```

**Output**: RViz updates the robot’s pose, showing simulated movement.

**Note**: Install the GUI if missing:

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui
```

## 3. LiDAR Visualization

Visualize LiDAR data (requires a physical LiDAR or ROS 2 bag file).

1. **Install Dependency**:

   ```bash
   sudo apt install libpcap-dev
   ```

2. **Build**:

   ```bash
   cd ~/lite3_ws
   colcon build --packages-up-to lslidar_driver --symlink-install
   ```

3. **Launch**:

   ```bash
   source ~/lite3_ws/install/setup.bash
   ros2 launch lite3_description lslidar.launch.py
   ```

**Output**: RViz shows point clouds (`/cx/lslidar_point_cloud`).

**Note**: Without hardware, use a ROS 2 bag file (see Section 4).

## 4. SLAM

Run SLAM algorithms for mapping and localization. Requires LiDAR and odometry data (physical robot or bag file).

### 4.1 Record ROS 2 Bag

Record data for SLAM testing:

```bash
source ~/lite3_ws/install/setup.bash
ros2 bag record /cx/scan /cx/lslidar_point_cloud /imu/data /leg_odom /tf /tf_static /joint_states /robot_description
```

**Note**: Without a robot, find a Lite3-compatible bag file or skip SLAM.

### 4.2 SLAM Toolbox

1. **Install**:

   ```bash
   sudo apt install ros-jazzy-slam-toolbox
   ```

2. **Launch**:

   ```bash
   source ~/lite3_ws/install/setup.bash
   ros2 launch lite3_description slam_toolbox.launch.py use_sim_time:=true
   ```

**Output**: RViz shows a map.

### 4.3 Cartographer 2D

1. **Install**:

   ```bash
   sudo apt install ros-jazzy-cartographer-ros
   ```

2. **Launch**:

   ```bash
   source ~/lite3_ws/install/setup.bash
   ros2 launch lite3_description slam_cartographer.launch.py use_sim_time:=true
   ```

### 4.4 Fast-LIO

1. **Install**: Follow instructions at FAST_LIO_ROS2.

   ```bash
   cd ~/lite3_ws/src
   git clone https://github.com/Ericsii/FAST_LIO_ROS2
   cd ~/lite3_ws
   rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:noble
   colcon build --packages-up-to fast_lio --symlink-install
   ```

2. **Launch**:

   ```bash
   source ~/lite3_ws/install/setup.bash
   ros2 launch lite3_description slam_fast_lio.launch.py use_sim_time:=true
   ```

### 4.5 GLIM

1. **Install**: Follow GLIM instructions. Use config at `lite3_description/config/glim`.

   ```bash
   cd ~/lite3_ws/src
   git clone https://github.com/koide3/glim
   cd ~/lite3_ws
   rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:noble
   colcon build --packages-up-to glim --symlink-install
   ```

2. **Launch**:

   ```bash
   source ~/lite3_ws/install/setup.bash
   ros2 launch lite3_description slam_fast_lio.launch.py use_sim_time:=true
   ```

## 5. Troubleshooting

- **rosbridge_server Not Found**:

  ```bash
  sudo apt install ros-jazzy-rosbridge-suite
  ```

  Or build from source (Section 1.3).

- **Build Errors**: Clean and retry:

  ```bash
  cd ~/lite3_ws
  rm -rf build install log
  colcon build --packages-up-to lite3_udp_bridge lite3_description --symlink-install
  ```

- **RViz Crashes**:

  ```bash
  sudo apt install ros-jazzy-rviz2
  ```

- **No Topics Without Robot**: Use RViz visualization (Section 2). For SLAM, acquire a bag file.

- **Workspace Conflicts**: Ensure only `lite3_ws` is sourced:

  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/lite3_ws/install/setup.bash
  ```

- **Joint State GUI Missing**:

  ```bash
  sudo apt install ros-jazzy-joint-state-publisher-gui
  ```

## 6. Notes

- **Simulation vs. Hardware**: RViz visualization works without a robot. The bridge requires a physical Lite3 for dynamic data.
- **Contributing**: Forked from legubiao/Lite3_ROS2. Submit pull requests to improve!
- **Support**: Open issues on your forked repo or contact the original author.
/////////////////////////////////////////////////////////::::::::::::::::::::::::::::::::::::::::::::::

# Lite3 ROS2

This repository is an unofficial project for DeepRobotics Lite3.

## 1. Quick Start

* prepare workspace

```bash
cd ~
mkdir ros2_ws
cd ros2_ws
mkdir src
cd src
```

* Clone the repository

```bash
git clone https://github.com/legubiao/Lite3_ROS2
cd lite3_ros2
git submodule update --init --recursive
```

* rosdep

```bash
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
```

* compile packages

```bash
cd ~/ros2_ws/
colcon build --packages-up-to lite3_udp_bridge lite3_description --symlink-install
```

* launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lite3_description bridge.launch.py
```

### 1.1 Visualize Robot

* To visualize and check the configuration of the robot in rviz, simply launch:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description visualize.launch.py check_gui:=true
  ```

![joints](.images/joints.png)

* To visualize the robot with the actual joint states data, launch:
  ```bash
  source ~/ysc_ws/install/setup.bash
  ros2 launch lite3_description visualize.launch.py
  ```

![odom](.images/odom.png)

### 1.2 LsLidar Visualization

* Install dependency
  ```bash
  sudo apt-get install libpcap-dev
  ```
* Compile lidar driver
  ```bash
  cd ~/ros2_ws/
  colcon build --packages-up-to lslidar_driver --symlink-install
  ```
* To visualize the robot with lidar data, launch:
  ```bash
  source ~/ros2_ws//install/setup.bash
  ros2 launch lite3_description lslidar.launch.py
  ```

![lidar](.images/lidar.png)

## 2. SLAM

* record ros2 bag
  ```bash
  ros2 bag record /cx/scan /cx/lslidar_point_cloud /imu/data /leg_odom /tf /tf_static /joint_states /robot_description
  ```

### 2.1 SLAM Toolbox

* Install
  ```bash
  sudo apt-get install ros-jazzy-slam-toolbox
  ```
* Launch
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description slam_toolbox.launch.py :use_sim_time:=true
  ```

![slamtoolbox](.images/slam_toolbox.png)

### 2.2 Cartographer 2D

* Install
  ```bash
  sudo apt-get install ros-jazzy-cartographer-ros
  ```
* Launch
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description slam_cartographer.launch.py :use_sim_time:=true
  ```

![cartographer2d](.images/cartographer2d.png)

### 2.3 Fast-LIO

* Install
    * ROS2 Verion of FAST_LIO could be found [here](https://github.com/Ericsii/FAST_LIO_ROS2)
* Launch
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description slam_fast_lio.launch.py :use_sim_time:=true
  ```

![fastlio](.images/fast_lio.png)

### 2.4 GLIM

* Install
    * Following the instruction [here](https://koide3.github.io/glim/)
    * The required config file could be found at `lite3_description/config/glim`
      
![glim](.images/glim.png)
