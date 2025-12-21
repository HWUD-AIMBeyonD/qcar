## Requirements

### Hardware
- Quanser QCar
- RPLiDAR (or compatible 2D LiDAR)

### Software
- Ubuntu (tested on NVIDIA Jetson–based environment)
- ROS 2 Dashing Diademata
- `slam_toolbox` (binary installation on Dashing)

> **Note:** Quanser proprietary libraries are required to run this stack but are not distributed in this repository. You must obtain and install them separately according to Quanser’s licensing terms.

---

## Installation

### 1. Clone the repository
```cd ~/qcar_ws/src```

```git clone <repository_url>```


### 2. Build the workspace
```cd ~/qcar_ws```

```colcon build --symlink-install```

```source install/setup.bash```

### 3. Install `slam_toolbox` (if not already installed)
```sudo apt update```

```sudo apt install ros-dashing-slam-toolbox```

---

## Running the System

### 1. Start QCar bringup

This script launches:
- **robot_description + RViz2** (runs without sudo)
- **Unified hardware interface node** (runs with sudo; handles LiDAR, odometry, and motor control)
  
```cd ~/qcar_ws```

```bash run_qcar_bringup.sh```


Once running, you should see:
- RViz2 with the QCar model and TF tree
- `/scan`, `/pointcloud`, `/odom`, `/tf` topics being published
- `/cmd_vel` available for sending velocity commands

---

## License

This project is licensed under the **Apache License 2.0**. See the `LICENSE` file for details.

diff --git a/README.md b/README.md
index c2d8d86ce93557b9df303f9fa6c407bfbb974932..16f872a5898d770613df11b04e738e99d495204b 100644
--- a/README.md
+++ b/README.md
@@ -1,60 +1,116 @@
+## QCar ROS 2 Workspace: Bringup, TF, and SLAM (ROS 2 Dashing)
+
+This repository contains a ROS 2 (Dashing) bringup + SLAM workflow for the **Quanser QCar**.  
+It is designed for research/education and focuses on reliable hardware access, correct TF, RViz visualization, and online SLAM using `slam_toolbox`.
+
+---
+
 ## Requirements
 
 ### Hardware
 - Quanser QCar
 - RPLiDAR (or compatible 2D LiDAR)
 
 ### Software
 - Ubuntu (tested on NVIDIA Jetson–based environment)
 - ROS 2 Dashing Diademata
 - `slam_toolbox` (binary installation on Dashing)
 
 > **Note:** Quanser proprietary libraries are required to run this stack but are not distributed in this repository. You must obtain and install them separately according to Quanser’s licensing terms.
 
 ---
 
+## QCar Core Modules (External)
+
+The ROS 2 nodes in this repo wrap Quanser’s QCar Python Core Modules. These modules are not included here, but the nodes expect them to be installed on the target system (for example under `/home/nvidia/Core Modules/Python`).
+
+Core modules used by the nodes:
+
+- **`product_QCar.py`**: Low-level QCar hardware I/O (motors, LEDs, encoder, IMU, battery/current).
+- **`q_essential.py`**: LIDAR and camera helpers (e.g., RPLIDAR, RealSense).
+- **`q_control.py`**, **`q_dp.py`**, **`q_interpretation.py`**, **`q_misc.py`**, **`q_ui.py`**: Control, decision/planning, perception helpers, utilities, and gamepad support.
+
+If you are extending the system, these are the most common integration points:
+- **Speed control** and **turn-speed limiting** are natural additions to the `/cmd_vel` path (teleop or hardware interface).
+- **Vision + interpretation** can be wrapped into a new ROS 2 node/package that publishes lane or obstacle topics.
+
+---
+
 ## Installation
 
 ### 1. Clone the repository
 ```cd ~/qcar_ws/src```
 
 ```git clone <repository_url>```
 
 
 ### 2. Build the workspace
 ```cd ~/qcar_ws```
 
 ```colcon build --symlink-install```
 
 ```source install/setup.bash```
 
 ### 3. Install `slam_toolbox` (if not already installed)
 ```sudo apt update```
 
 ```sudo apt install ros-dashing-slam-toolbox```
 
 ---
 
-## Running the System
+## Quick Start Summary (Read This First)
 
-### 1. Start QCar bringup
+### What you run (normal workflow)
 
-This script launches:
-- **robot_description + RViz2** (runs without sudo)
-- **Unified hardware interface node** (runs with sudo; handles LiDAR, odometry, and motor control)
-  
-```cd ~/qcar_ws```
+#### 1) Bring up the robot (drivers + RViz + TF)
+```bash
+cd ~/qcar_ws
+bash run_qcar_bringup.sh
+```
+
+#### 2) Start SLAM (in another terminal)
+```bash
+source /opt/ros/dashing/setup.bash
+source ~/qcar_ws/install/setup.bash
+ros2 launch slam_toolbox online_async_launch.py params_file:=~/qcar_ws/qcar_slam.yaml
+```
+
+---
+
+## System Architecture
+
+### Split-permission setup (this is intentional)
+
+- **Hardware-touching node runs with sudo.** Quanser HIL requires privileged access and cannot be safely delegated via udev/groups in this setup.
+- **Visualization + URDF + RViz runs as normal user.** RViz breaks under sudo due to display/X11 permissions.
+
+### Unified hardware interface node (single point of hardware access)
+
+- **Package:** `qcar_nav2_bringup`  
+- **Node:** `qcar_hardware_interface` (runs with sudo)
+
+**Publishes:**
+- `/scan` (`sensor_msgs/LaserScan`)
+- `/pointcloud` (`sensor_msgs/PointCloud`)
+- `/odom` (`nav_msgs/Odometry`)
+- `TF: odom → base` (dynamic)
+
+**Subscribes:**
+- `/cmd_vel` (`geometry_msgs/Twist`)
 
-```bash run_qcar_bringup.sh```
+**Why this matters:** only **one** node should initialize the QCar HIL card. Running multiple hardware nodes commonly causes:
+- “GPIO is in use by another application”
+- invalid card handle / HIL initialization errors
 
+### Visualization components (no sudo)
 
-Once running, you should see:
-- RViz2 with the QCar model and TF tree
-- `/scan`, `/pointcloud`, `/odom`, `/tf` topics being published
-- `/cmd_vel` available for sending velocity commands
+- **Package:** `robot_description`
+- **Runs:**
+  - `robot_state_publisher` (URDF → TF)
+  - `RViz2` (visualization)
 
 ---
 
 ## License
 
 This project is licensed under the **Apache License 2.0**. See the `LICENSE` file for details.

