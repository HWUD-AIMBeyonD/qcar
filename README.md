# QCar ROS 2 Workspace: Bringup, TF, SLAM, and Nav2 (ROS 2 Dashing)

This repository contains a ROS 2 (Dashing) bringup workflow for the **Quanser QCar** with:

* Reliable hardware access (Quanser Core Modules)
* Correct TF tree and RViz visualization
* Online SLAM using `slam_toolbox`
* **Custom Navigation Architecture:**
  * **Regulated Pure Pursuit Controller (RPP)**: Backported modern controller logic running in a custom standalone server.
  * Legacy support for **DWB Controller** (configurable).

> This is designed for research/education on real hardware. Nav2 will command the vehicle when a goal is set.

---

## Requirements

### Hardware

* Quanser QCar
* RPLiDAR (or compatible 2D LiDAR)

### Software

* Ubuntu (tested on NVIDIA Jetson–based environment)
* ROS 2 Dashing Diademata
* `slam_toolbox` (binary installation on Dashing)
* Nav2 packages (Dashing)

> **Note:** Quanser proprietary libraries are required to run this stack but are not distributed in this repository. You must obtain and install them separately according to Quanser's licensing terms.

---

## QCar Core Modules (External)

The ROS 2 nodes in this repo wrap Quanser's QCar Python Core Modules. These modules are not included here, but the nodes expect them to be installed on the target system (for example under `/home/nvidia/Core Modules/Python`).

Core modules used by the nodes:

* **`product_QCar.py`**: Low-level QCar hardware I/O (motors, LEDs, encoder, battery/current).
* **`q_essential.py`**: LIDAR and camera helpers (e.g., RPLIDAR, RealSense).
* **`q_control.py`**, **`q_dp.py`**, **`q_interpretation.py`**, **`q_misc.py`**, **`q_ui.py`**: Control, decision/planning, perception helpers, utilities, and gamepad support.

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
  ros-dashing-nav2-recoveries \
  ros-dashing-nav2-map-server \
  ros-dashing-nav2-amcl
```

---

## Repository Layout

Key packages and files:

* **`rpp_controller/`**
  * Source code for the **Custom Controller Server**.
  * Contains the backported Regulated Pure Pursuit algorithm and the action server implementation that replaces the standard Dashing `dwb_controller`.

* **`qcar_nav2_bringup/`**
  * `qcar_hardware_interface.py` — publishes `/scan`, `/pointcloud`, `/odom`, TF; subscribes `/cmd_vel`
  * `launch/`
    * `qcar_bringup.launch.py` — hardware + robot_description (optional)
    * `slam_launch.launch.py` — SLAM Toolbox online bringup (async)
    * `localization.launch.py` — **Localization Mode**: Runs Map Server + AMCL (Requires saved map).
    * `nav2_online.launch.py` — **Nav2 bringup**. Configured to switch between Custom RPP and Legacy DWB.
  * `config/`
    * `slam_toolbox_online.yaml` — slam_toolbox params
    * `nav2_params.yaml` — **Hybrid Param File**. Contains two sections:
      * **Option A:** RPP Controller + Nested Local Costmap 
      * **Option B:** Legacy DWB Controller + Standalone Local Costmap 

---

## Quick Start Summary

You can choose between **Mapping (SLAM)** or **Map-Based Navigation**.

### Workflow A: Mapping (SLAM)

*Use this to create a new map.*

1. **Bring up the robot**
```bash
cd ~/qcar_ws
bash run_qcar_bringup.sh
```

2. **Start SLAM** (in another terminal)
```bash
source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash
ros2 launch qcar_nav2_bringup slam_launch.launch.py
```

3. **Save Map**
When finished, save the map using `nav2_map_server`.

---

### Workflow B: Map-Based Navigation

*Use this to drive autonomously on a saved map.*

1. **Bring up the robot**
```bash
cd ~/qcar_ws
bash run_qcar_bringup.sh
```

2. **Start Localization** (Map Server + AMCL)
```bash
source ~/qcar_ws/install/setup.bash
ros2 launch qcar_nav2_bringup localization.launch.py map:=/home/nvidia/qcar_ws/maps/latest_map.yaml
```

3. **Start Nav2** (Planner + Controller)
```bash
source ~/qcar_ws/install/setup.bash
ros2 launch qcar_nav2_bringup nav2_online.launch.py
```

4. **Set Initial Pose**
AMCL requires an initial pose estimate. You can do this via RViz ("2D Pose Estimate") or via terminal:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

5. **Send a Goal**
**Option 1: Using RViz**
Use the **"2D Nav Goal"** tool in RViz to click and drag a destination on the map.

**Option 2: Using Terminal**
Send the robot to specific coordinates (e.g., x=1.0 meters, y=0.0 meters).
```bash
ros2 action send_goal /NavigateToPose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

---

### Expected outcome

* Hardware publishes `/scan` and `/odom`.
* SLAM/Map Server publishes `/map`.
* **Nav2 Controller**:
  * Launches `rpp_controller_server`.
  * Creates a Local Costmap on topic `/costmap` (internal to the controller).
  * Publishes `/cmd_vel` when a goal is set in RViz.

> **Tip:** If the robot moves too slowly or stalls, check `min_x_velocity_threshold` and `desired_linear_vel` in `nav2_params.yaml` (Option A). QCar static friction requires higher minimum speeds (~0.6 m/s) to start moving.

---

## TF Frames (Critical)

This stack expects a single connected TF tree:

* `map → odom` (published by `slam_toolbox`)
* `odom → base` (published by `qcar_hardware_interface`)
* `base → lidar` (published by URDF / robot_state_publisher)
* `base → base_footprint` (published by URDF / robot_state_publisher)
* `base → base_link` (published by URDF / robot_state_publisher)

### Verifying TF tree
```bash
ros2 run tf2_ros tf2_echo map odom      # SLAM
ros2 run tf2_ros tf2_echo odom base     # Odometry
```

---

## System Architecture

### 1. Split-permission setup (intentional)

* **Hardware-touching node runs with sudo.** Quanser HIL requires privileged access.
* **URDF + RViz runs as normal user.**

### 2. Custom Navigation Architecture (RPP vs DWB)

A **Custom Controller Server** has been implemented to bypass limitations in ROS 2 Dashing's `nav2_dwb_controller`.

* **Regulated Pure Pursuit**
  * Runs inside `rpp_controller_server`.
  * This custom node manages its own internal **Local Costmap** instance.
  * It handles path-following using pure pursuit geometry, which is smoother and more robust for car-like robots than the default DWB.
  * **Note:** This node is *not* managed by the Nav2 Lifecycle Manager (it auto-starts).

* **Option B (Legacy): DWB Controller**
  * Uses standard `nav2_costmap_2d` (standalone node) + `dwb_controller`.
  * Available as a fallback by commenting/uncommenting sections in `nav2_online.launch.py` and `nav2_params.yaml`.

### 3. Unified hardware interface node

* **Node:** `qcar_hardware_interface` (runs with sudo)
* **Publishes:** `/scan`, `/pointcloud`, `/odom`, TF `odom → base`
* **Subscribes:** `/cmd_vel`

---

## License

This project is licensed under the Apache License 2.0. See the `LICENSE` file for details.
