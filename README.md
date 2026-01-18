# QCar ROS 2 Workspace: Bringup, TF, SLAM, and Nav2 (ROS 2 Dashing)

This repository contains a complete ROS 2 (Dashing) bringup workflow for the **Quanser QCar** featuring a fully custom, optimized navigation stack designed for legacy hardware (Jetson Nano).

**Key Features:**

* Reliable hardware access (Quanser Core Modules)
* Correct TF tree and RViz visualization
* Online SLAM using `slam_toolbox`
* **Custom Integrated Navigation Architecture:**
* **Custom Hybrid A* Planner**: A kinematic-aware global planner running in a standalone server with an integrated global costmap.
* **Regulated Pure Pursuit Controller (RPP)**: A smooth path-following controller running in a standalone server with an integrated local costmap.
* *Note: Both custom servers use an "Integrated Architecture" (embedding costmaps directly into the nodes) to maximize performance on the Jetson Nano.*



> This stack is designed for research/education. It replaces standard Nav2 nodes (NavFn/DWB) with custom implementations to solve Dashing-specific stability and performance issues.

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
  ros-dashing-nav2-bt-navigator \
  ros-dashing-nav2-lifecycle-manager \
  ros-dashing-nav2-recoveries \
  ros-dashing-nav2-map-server \
  ros-dashing-nav2-amcl

```

---

## Repository Layout

Key packages and files:

* **`custom_hybrid_planner/`**
* Source code for the **Custom Planner Server**.
* Implements a Hybrid A* algorithm that respects the QCar's minimum turning radius.
* Manages its own internal **Global Costmap** to reduce inter-process communication overhead.


* **`rpp_controller/`**
* Source code for the **Custom Controller Server**.
* Implements the Regulated Pure Pursuit algorithm for smooth path tracking.
* Manages its own internal **Local Costmap**.


* **`qcar_nav2_bringup/`**
* `qcar_hardware_interface.py` — publishes `/scan`, `/pointcloud`, `/odom`, TF; subscribes `/cmd_vel`
* `launch/`
* `qcar_bringup.launch.py` — hardware + robot_description (optional)
* `slam_launch.launch.py` — SLAM Toolbox online bringup (async)
* `localization.launch.py` — **Localization Mode**: Runs Map Server + AMCL (Requires saved map).
* `nav2_online.launch.py` — **Nav2 Bringup**. Launches the full stack: Hybrid Planner, RPP Controller, BT Navigator, and Lifecycle Manager.


* `config/`
* `slam_toolbox_online.yaml` — slam_toolbox params
* `nav2_params.yaml` — **Integrated Param File**. Contains configuration for:
* `hybrid_planner_server` (and its internal global costmap)
* `controller_server` (and its internal local costmap)
* `bt_navigator`, `amcl`, etc.







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

3. **Start Nav2** (Custom Planner + Controller)

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

* *Note: You must manually add the custom costmap topics in RViz to see them (see Visualization section).*

**Option 2: Using Terminal**
Send the robot to specific coordinates (e.g., x=1.0 meters, y=0.0 meters).

```bash
ros2 action send_goal /NavigateToPose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"

```

---

## Visualization (RViz)

Because this stack uses an **Integrated Architecture** (costmaps inside nodes), the default Nav2 RViz config will not show the maps. You must manually add these topics in RViz:

* **Global Costmap:** `/hybrid_planner_server/global_costmap/costmap`
* **Local Costmap:** `/controller_server/local_costmap/costmap`
* **Path:** The custom planner does not publish a visualization topic by default (performance optimization).

---

## System Architecture

### 1. Integrated Architecture

To improve performance on the Jetson Nano and solve ROS 2 Dashing lifecycle issues, this stack does **not** use standalone `nav2_costmap_2d` nodes.

* **Hybrid Planner Server:** Manages the Global Costmap internally in a separate thread.
* **RPP Controller Server:** Manages the Local Costmap internally in a separate thread.
* **Lifecycle:** Both custom nodes are **Auto-Starting** and are NOT managed by the `lifecycle_manager` (to prevent configuration hangs).

### 2. Topic Remapping

The launch file automatically handles remapping to ensure the custom nodes connect to the standard Nav2 interfaces:

* **Map:** `/hybrid_planner_server/map` → `/map`
* **Action:** `/hybrid_planner_server/ComputePathToPose` → `/ComputePathToPose`

### 3. Unified Hardware Interface

* **Node:** `qcar_hardware_interface` (runs with sudo)
* **Publishes:** `/scan`, `/pointcloud`, `/odom`, TF `odom → base`
* **Subscribes:** `/cmd_vel`

---

## License

This project is licensed under the Apache License 2.0. See the `LICENSE` file for details.
