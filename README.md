# QCar ROS 2 Workspace: Bringup, TF, SLAM, and Nav2 (ROS 2 Dashing)

This repository contains a ROS 2 (Dashing) bringup workflow for the **Quanser QCar** with:
- Reliable hardware access (Quanser Core Modules)
- Correct TF tree and RViz visualization
- Online SLAM using `slam_toolbox`
- Online navigation using **Nav2** (planner + costmaps + DWB controller)

> This is designed for research/education on real hardware. Nav2 will command the vehicle when a goal is set.

---

## Requirements

### Hardware
- Quanser QCar
- RPLiDAR (or compatible 2D LiDAR)

### Software
- Ubuntu (tested on NVIDIA Jetson–based environment)
- ROS 2 Dashing Diademata
- `slam_toolbox` (binary installation on Dashing)
- Nav2 packages (Dashing)

> **Note:** Quanser proprietary libraries are required to run this stack but are not distributed in this repository. You must obtain and install them separately according to Quanser's licensing terms.

---

## QCar Core Modules (External)

The ROS 2 nodes in this repo wrap Quanser's QCar Python Core Modules. These modules are not included here, but the nodes expect them to be installed on the target system (for example under `/home/nvidia/Core Modules/Python`).

Core modules used by the nodes:
- **`product_QCar.py`**: Low-level QCar hardware I/O (motors, LEDs, encoder, battery/current).
- **`q_essential.py`**: LIDAR and camera helpers (e.g., RPLIDAR, RealSense).
- **`q_control.py`**, **`q_dp.py`**, **`q_interpretation.py`**, **`q_misc.py`**, **`q_ui.py`**: Control, decision/planning, perception helpers, utilities, and gamepad support.

Common extension points:
- Add speed limiting / smoothing on `/cmd_vel`
- Add a perception node (lane/obstacles) publishing custom topics for future planners/controllers

---

## Installation

### 1. Clone the repository
```bash
cd ~/qcar_ws/src
git clone <repository_url>
```

### 2. Build the workspace
```bash
cd ~/qcar_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Install SLAM + Nav2 (if not already installed)
```bash
sudo apt update
sudo apt install ros-dashing-slam-toolbox
sudo apt install ros-dashing-navigation2 ros-dashing-nav2-bringup
```

If you are missing specific Nav2 components on Dashing, install these as needed:
```bash
sudo apt install \
  ros-dashing-nav2-costmap-2d \
  ros-dashing-nav2-navfn-planner \
  ros-dashing-nav2-dwb-controller \
  ros-dashing-nav2-bt-navigator \
  ros-dashing-nav2-lifecycle-manager \
  ros-dashing-nav2-recoveries
```

---

## Repository Layout

Key files (kept inside `qcar_nav2_bringup` as a single bringup package):

* `qcar_nav2_bringup/`
  * `qcar_hardware_interface.py` — publishes `/scan`, `/pointcloud`, `/odom`, TF; subscribes `/cmd_vel`
* `launch/`
  * `qcar_bringup.launch.py` — hardware + robot_description (optional)
  * `slam_launch.launch.py` — SLAM Toolbox online bringup (async)
  * `nav2_online.launch.py` — Nav2 online bringup (planner + costmaps + DWB)
* `config/`
  * `slam_toolbox_online.yaml` — slam_toolbox params
  * `nav2_params.yaml` — Nav2 params (frames, costmaps, DWB tuning)

---

## Quick Start Summary 

### 1) Bring up the robot (drivers + RViz + TF)
```bash
cd ~/qcar_ws
bash run_qcar_bringup.sh
```

This should start:
* `robot_state_publisher` + RViz (non-sudo)
* `qcar_hardware_interface` (sudo, single point of hardware access)

### 2) Start SLAM (in another terminal)
```bash
source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash
ros2 launch qcar_nav2_bringup slam_launch.launch.py
```

### 3) Start Nav2 (in another terminal)
```bash
source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash
ros2 launch qcar_nav2_bringup nav2_online.launch.py
```

### Expected outcome
* Hardware publishes `/scan` and `/odom`, plus TF `odom → base`
* SLAM publishes `/map` and TF `map → odom`
* Nav2 creates global + local costmaps and publishes `/cmd_vel` when a goal is set in RViz

---

## TF Frames (Critical)

This stack expects a single connected TF tree:
* `map → odom` (published by `slam_toolbox`)
* `odom → base` (published by `qcar_hardware_interface`)
* `base → lidar` (published by URDF / robot_state_publisher)
* `base → base_footprint` (published by URDF / robot_state_publisher)
* `base → base_link` (published by URDF / robot_state_publisher)

### Base frame naming

* **Primary robot base frame:** `base` (used by URDF + odometry)
* **Nav2 compatibility frames:** `base_link` and `base_footprint` (both children of `base`, coincident)

All three frames (`base`, `base_link`, `base_footprint`) are at the same location via fixed joints in the URDF. This ensures compatibility with:
- SLAM Toolbox (expects `base_footprint`)
- Nav2 components (may default to `base_link`)
- Your existing system (built around `base`)

### Verifying TF tree

Check that all frames are connected:
```bash
# Check complete chain from map to lidar
ros2 run tf2_ros tf2_echo map lidar

# Verify individual links
ros2 run tf2_ros tf2_echo map odom      # SLAM
ros2 run tf2_ros tf2_echo odom base     # Odometry
ros2 run tf2_ros tf2_echo base lidar    # URDF (static)
ros2 run tf2_ros tf2_echo base base_link         # URDF (static)
ros2 run tf2_ros tf2_echo base base_footprint    # URDF (static)
```

> **Note:** Dashing may print warnings like `LOOPING due to no latching` for static transforms — this is expected behavior and does not affect functionality.
---

## System Architecture

### Split-permission setup (intentional)
* **Hardware-touching node runs with sudo.** Quanser HIL requires privileged access.
* **URDF + RViz runs as normal user.** RViz often breaks under sudo due to display/X11 permissions.

### Unified hardware interface node (single point of hardware access)
* **Package:** `qcar_nav2_bringup`
* **Node:** `qcar_hardware_interface` (runs with sudo)

**Publishes:**
* `/scan` (`sensor_msgs/LaserScan`)
* `/pointcloud` (`sensor_msgs/PointCloud`)
* `/odom` (`nav_msgs/Odometry`)
* TF: `odom → base` (dynamic)

**Subscribes:**
* `/cmd_vel` (`geometry_msgs/Twist`)

**Why this matters:** only one node should initialize the QCar HIL card. Running multiple hardware nodes commonly causes:
* "GPIO is in use by another application"
* invalid card handle / HIL initialization errors

### Visualization components (no sudo)
* **Package:** `robot_description`
* **Runs:**
  * `robot_state_publisher` (URDF → TF)
  * `RViz2` (visualization)

---

## License

This project is licensed under the Apache License 2.0. See the `LICENSE` file for details.
