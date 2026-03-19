# LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario

This project implements a distributed robotic system for automated box sorting.  
It integrates a Pioneer P3-DX mobile base, a Dobot CR3 robotic arm, and a LiDAR-based volume estimation system, all orchestrated via ROS 2.

---

## 🏗 System Architecture

The system is distributed across four processing units to balance the computational load:

| Hardware                 | OS           | ROS 2   | Responsibility                          |
|--------------------------|--------------|---------|-----------------------------------------|
| PC 1 (Arm Control)       | Ubuntu 22.04 | Humble  | Dobot CR3 Control & Logic               |
| PC 2 (Vision processing) | Ubuntu 22.04 | Humble  | LiDAR PointCloud Processing (Open3D)    |
| PC 3 (Trajectory Control)| Ubuntu 22.04 | Humble  | Pioneer P3-DX Control & Logic           |
| Jetson Nano              | Ubuntu 20.04 | Foxy    | Pioneer P3-DX Driver control            |
| Raspberry Pi 3           | Ubuntu 20.04 | Foxy    | Unitree LiDAR & Gripper Driver control  |

---

## 🛠 Prerequisites & Installation

1. **Robotic Arm (PC 1)**
   - Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
   - Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
   - Install [Dobot ROS2 V3 Driver](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3/blob/main/README_EN.md)
   - Download the folder CR3_Arm-Control from this repository

2. **Vision & Processing (PC 2)**
   - Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
   - Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
   - Download the folder Lidar_4D-Processing from this repository
   - Install Open3D: `pip3 install open3d`.

3. **Mobile Robot Control trajectory**
   - Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
   - Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
   - Download the folder Pioneer_3DX-Control from this repository
  
4. **Mobile Base (Jetson Nano)**
    - Install [Ubuntu 20.04](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file)
    - Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
    - Install RosAria Driver [adapted for ROS 2](https://github.com/Bessawy/RosAria2)
    - Connect via Serial-to-USB to the Pioneer.

5. **Sensors & Actuators (Raspberry Pi 3)**
   - Install [Ubuntu 20.04](https://github.com/Joshua-Riek/ubuntu-raspberry-pi) or install Ubuntu 22.04 and Docker with         Ubuntu 20.04.
   - Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
   - Install Unitree [LiDAR SDK ROS2](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/README.md)
   - Install RPi.GPIO for relay control.
   - Download the folder Gripper-Control from this repository
---

## 🚀 Execution Sequence

To ensure the handshakes between nodes occur correctly, follow this specific launch order.  
**Note:** Every terminal must have the ROS 2 environment sourced (`source /opt/ros/<distro>/setup.bash`) and export ROS_DOMAIN_ID=0. Make sure that all devices are on the same network. Adjust the settings of the waypoints of the mobile robot, the manipulator robot, and the Lidar according to your environment, in the parameters of the control and processing codes. For more information, read the article to which this repository is linked.

  ### Step 1: Robotic Arm Controller and Driver Nodes (PC 1)
  
  - Load the Dobot workspace and start the driver node at a terminal
    ```
    source ~/dobot_ws/install/setup.bash
    ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
    ```
  - In other Terminal run controller node
    ```
    source ~/dobot_ws/install/setup.bash
    python3 box_organizer_final.py
    ```

  ### Step 2: Gripper and Lidar Driver Nodes (Raspberry Pi - SSH)
    Connect via SSH and start the relay node at a terminal:
    ```bash
    python3 GripperController.py
    In other Terminal run lidar driver node:
    ```bash
    source  ~unilidar_sdk/unitree_lidar_ros2/install/setup.bash
    ros2 launch unitree_lidar_ros2 launch.py

  ### Step 3: LiDAR Processing Node (PC 2)
    Run the volume estimation script:
    ```bash
    python3 PCDVolumeEstimationBoundingBox.py

   ### Step 4: Pioneer Node Driver (Jetson Nano)
    Start the mobile driver node:
    ```bash
    source ~/rosaria_ws/install/setup.bash

   ### Step 4: Pioneer 3DX Controller Node (PC 3)
     Run the volume estimation script:
     ```bash
     python3 Bezier_control.py

---
## 🔄 Communication Flow (The "Handshake")

- Pioneer reaches the docking station and sends a `"measure"` request to `/volume_request`.  
- LiDAR captures the point cloud, processes the dimensions, and sends data to `/volume_topic`.  
- Dobot CR3 receives the volume, calculates the sorting slot, and performs the Pick-and-Place.  
- Gripper is triggered via `/garra_command` during the process.  
- Dobot CR3 sends a `"start"` signal to `/string` to release the Pioneer for the next box.  

---

## 📝 Authors

- **Luciano Moreira** - (UFV / IF Sudeste MG)  
- **Jose Leandro Caires Mirante** - (UFV)
- **Geissiane Aguiar** - (UFV)
- **Alexandre Santos Brandão** - (UFV)

