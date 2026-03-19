# LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario

This project implements a distributed robotic system for automated box sorting.  
It integrates a Pioneer P3-DX mobile base, a Dobot CR3 robotic arm, and a LiDAR-based volume estimation system, all orchestrated via ROS 2.

---

## 🏗 System Architecture

The system is distributed across four processing units to balance the computational load:

| Hardware                 | OS            | ROS 2   | Responsibility                         |
|--------------------------|---------------|---------|----------a------------------------------|
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
   - Ensure the Controller CR3 is reachable via Ethernet.

2. **Vision & Processing (PC 2)**
   - Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
   - Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
   - Download the folder Lidar_4D-Processing from this repository
   - Install Open3D: `pip3 install open3d`.

3. **Mobile Robot Control trajectory**
   - Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
   - Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
   - Download the folder Pioneer_3DX-Control from this repository
   - Ensure the Controller CR3 is reachable via Ethernet.
  
4.  **Mobile Base (Jetson Nano)**
   - Install [Ubuntu 20.04](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file)
   - Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
   - Install RosAria Driver [adapted for ROS 2](https://github.com/Bessawy/RosAria2)
   - Connect via Serial-to-USB to the Pioneer.

5. **Sensors & Actuators (Raspberry Pi 3)**
   - Install [Ubuntu 20.04](https://github.com/Joshua-Riek/ubuntu-raspberry-pi) or install Ubuntu 22.04 and Docker with         Ubuntu 20.04.
   - Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
   - Install Unitree [LiDAR SDK ROS2](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/README.md)
   - Install RPi.GPIO for relay control.
---

## 🚀 Execution Sequence

To ensure the handshakes between nodes occur correctly, follow this specific launch order.  
**Note:** Every terminal must have the ROS 2 environment sourced (`source /opt/ros/<distro>/setup.bash`).

  ### Step 1: Robotic Arm (PC 1)
    Load the Dobot workspace and start the controller:
    ```bash
    source ~/dobot_ws/install/setup.bash
    python3 box_organizer_final.py

  ### Step 2: Gripper Control (Raspberry Pi - SSH)
    Connect via SSH and start the relay node:
    ```bash
    python3 GripperController.py

  ### Step 3: LiDAR Processing (PC 2)
    Run the volume estimation script:
    ```bash
    python3 PCDVolumeEstimationBoundingBox.py

  ### Step 4: Pioneer Controller (Jetson Nano)
    Start the mobile base logic:
    ```bash
    source ~/rosaria_ws/install/setup.bash
    python3 pioneer_controller_Bezier_v7.py
---
## 🔄 Communication Flow (The "Handshake")

- Pioneer reaches the docking station and sends a `"measure"` request to `/volume_request`.  
- LiDAR captures the point cloud, processes the dimensions, and sends data to `/volume_topic`.  
- Dobot CR3 receives the volume, calculates the sorting slot, and performs the Pick-and-Place.  
- Gripper is triggered via `/garra_command` during the process.  
- Dobot CR3 sends a `"start"` signal to `/string` to release the Pioneer for the next box.  

---

## 📝 Authors and Acknowledgments

- **Jose Leandro Caires Mirante** - Lead Developer (UFV / IF Sudeste MG)  
- **Luciano Moreira** - Hardware & Gripper Integration  

