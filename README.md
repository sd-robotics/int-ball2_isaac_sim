# Int-Ball2 Simulator (Isaac Sim)

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![日本語版 README](https://img.shields.io/badge/日本語-d9d9d9)](./README_JA.md)

![GitHub contributors](https://img.shields.io/github/contributors/sd-robotics/int-ball2_simulator)
![GitHub issues](https://img.shields.io/github/issues/sd-robotics/int-ball2_simulator)
![GitHub fork](https://img.shields.io/github/forks/sd-robotics/int-ball2_simulator)
![GitHub stars](https://img.shields.io/github/stars/sd-robotics/int-ball2_simulator)

<!--  [![Ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) -->
<!-- [![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-green.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) -->
<!-- [![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html) -->
<!-- [![ros2-humble installation](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) -->

<p style="display: inline">
  <img src="https://img.shields.io/badge/-Ubuntu_22.04_LTS-555555.svg?style=flat&logo=ubuntu">  
  <img src="https://img.shields.io/badge/-Isaac_Sim 4.2.0-76B900.svg?style=flat&logo=nvidia&logoColor=white">
  <img src="https://img.shields.io/badge/-ROS2 Humble-%2322314E?style=flat&logo=ROS&logoColor=white">
  <img src="https://img.shields.io/badge/-Python 3.10-3776AB.svg?logo=python&style=flat&logoColor=white">
  <img src="https://img.shields.io/badge/License-Apache--2.0-60C060.svg?style=flat">
</p>

![Int-Ball2 Isaac Sim 01](img/int-ball2_isaac_sim_01.png)

## Table of Contents
1. [**What is Int-Ball2 Simulator (Isaac Sim)**](#what-is-int-ball2-simulator-isaac-sim)

2. [**Prerequisites**](#prerequisites)

3. [**Installation**](#installation)
    1. [Clone Repository](#clone-repository)
    2. [Install Dependencies](#install-dependencies)
    3. [Download Assets](#download-assets)

4. [**Usage**](#usage)
    1. [Build & Source](#build--source)
    2. [Launch the Simulation](#launch-the-simulation)
    3. [How to control Int-Ball2](#how-to-control-int-ball2)
    4. [Teleoperation (Joy Controller)](#teleoperation-joy-controller)
    5. [Data Visulisation](#data-visualisation)

5. [**Data Visualisation**](#data-visualisation)

6. [**Acknowledgement**](#acknowledgement)

---

## What is Int-Ball2 Simulator (Isaac Sim)
Int-Ball2 is a free-flying camera robot deployed in the ISS Japan Experimental Module (JEM). It is remotely controlled from the ground to capture video images and support astronauts. Additionally, Int-Ball2 can run user-developed software as an extended functionality and can be used as a platform for demonstrating robotic technology in space.

This Repository provides a ROS + Isaac Sim (NVIDIA) based simulator for Int-Ball2. It simulates Int-Ball2's behavior in the ISS/JEM environment with user-developed programs.

![Int-Ball2 Hardware](img/int-ball2_hardware.png)

## Prerequisites
In order to use this project, you need to get ready the following environment.

|  Package  |         Version         |
| --------- | ----------------------- |
|   Ubuntu  | 22.04 (Jammy Jellyfish) |
| Isaac Sim | 4.2.0 (September 2024)  |
|    ROS    |     Humble Hawksbill    |
|   Python  |          3.10 <=        |

## Installation
### Clone Repository
Make a workspace if you do not have one already.
```bash
mkdir -p ~/int-ball2_ws/src
cd ~/int-ball2_ws/src
```

Clone the packages to your workspace.
```bash
git clone https://github.com/sd-robotics/int-ball2_isaac_sim.git
```

### Install Dependencies
Update the list of available packages (Isaac ROS).
```bash
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
```

Move into the workspace.
```bash
cd ~/int-ball2_ws/
```

Install the dependencies.
``` bash
rosdep install --from-paths src --ignore-src -r -y
```

### Download Assets
Run the following command to download the assets.
Move into the project.
```bash
cd ~/int-ball2_ws/src/int-ball2_isaac_sim
```

Download the assets.
```bash
bash install_local.sh
```

## Usage
### Build & Source
Build and Setup the package.
```bash
cd ~/int-ball2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch the Simulation
Launch the simulation by ros2 launch.
```bash
ros2 launch int-ball2_isaac_sim int-ball2_isaac_sim.launch.py gui:="~/int-ball2_ws/src/int-ball2_isaac_sim/assets/KIBOU.usd"
```

![Int-Ball2 Isaac Sim 02](img/int-ball2_isaac_sim_02.png)

Change of perspective to look around the ISS Kibo (Japanese Experiment Module).

![Int-Ball2 Isaac Sim 02](img/int-ball2_isaac_sim_03.png)

> [!TIP]
> If you are using laptop to run Isaac Sim and suffering the problem of monitor freezing when Isaac Sim is launched, you might want to switch the system to use the NVIDIA GPU by the following command.
> ```bash
> sudo prime-select nvidia
> ```
>
> This will result in a better performance in graphic-intensive tasks. To check if your laptop has successfully switched to NVIDIA GPU, you can use the command.
> ```bash
> prime-select query
> ```

### How to control Int-Ball2
The following data can be obtained by the user program.

| Type  |    ROS Definition Name    |                                      Overview                                         |
| ----- | ------------------------- | ------------------------------------------------------------------------------------- | 
| Topic | /camera_main/image_raw    | Image of the main camera on the front of the Int-Ball2.                               |
| Topic | /camera_main/camera_info  | Information about the main camera on the front of Int-Ball2.                          |
| Topic | /camera_left/image_raw    | Image of the sub-camera on the left side of Int-Ball2 (left).                         |
| Topic | /camera_left/camera_info  | Information on the sub-camera on the left side of Int-Ball2 (left).                   |
| Topic | /camera_right/image_raw   | Image of the sub-camera on the left side of Int-Ball2 (right).                        |
| Topic | /camera_right/camera_info | Information on the sub-camera on the left side of Int-Ball2 (right).                  |
| Topic | /imu/imu                  | Sensor value of the IMU(Inertial Measurement Unit).                                   |
| Topic | /ground_truth             | True value of the robot position and orientation (docking station to Int-Ball2 body). |

The following data can be controlled by the user program.

| Type  |    ROS Definition Name    |                                            Overview                                             |
| ----- | ------------------------- | ----------------------------------------------------------------------------------------------- | 
| Topic | /ctl/wrench               | Input values of the Force and Torque to Int-Ball2. Int-Ball2 is controlled by Force and Torque. |

### Teleoperation (Joy Controller)
Setup the package.
```bash
cd ~/int-ball2_ws
source install/setup.bash
```

Make sure that you have joystick controller connected to the PC before running the command.
Then Launch the teleop (operated by joystick controller).
```bash
ros2 launch int-ball2_control int-ball2_teleop.launch.py
```

Operation with the joystick controller is as follows.
Left stick for X-axis and Y-axis translational movement, B button + RT or LT for Z-axis translational movement.
Right stick for X-axis and Y-axis rotational movement, A button + RT or LT for Z-axis translational movement.

![Int-Ball2 Teleop](img/int-ball2_teleop.png)

## Data Visualisation
Setup the package.
```bash
cd ~/int-ball2_ws
source install/setup.bash
```

Launch the Rviz.
```bash
ros2 launch int-ball2_control rviz_visualize.launch.py 
```

![Int-Ball2 Rviz](img/int-ball2_rviz.png)

## Acknowledgement
This simulator was developed by Space Data in cooperation with JAXA within the framework of the Space Innovation Partnership (J-SPARC: JAXA Space Innovation through Partnership and Co-creation).

> [!TIP]
> This document includes content from JAXA's Int-Ball2 Simulator, which is licensed under the Apache License 2.0.
> - [Int-Ball2 Simulator (Gazebo)](https://github.com/jaxa/int-ball2_simulator)

---

[Back to Top](#int-ball2-simulator)
