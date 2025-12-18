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
