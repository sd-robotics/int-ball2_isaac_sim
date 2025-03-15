# Int-Ball2 Simulator (Isaac Sim)

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![日本語版 README](https://img.shields.io/badge/日本語-d9d9d9)](./README_JA.md)

![GitHub contributors](https://img.shields.io/github/contributors/sd-robotics/int-ball2_simulator)
![GitHub issues](https://img.shields.io/github/issues/sd-robotics/int-ball2_simulator)
![GitHub fork](https://img.shields.io/github/forks/sd-robotics/int-ball2_simulator)
![GitHub stars](https://img.shields.io/github/stars/sd-robotics/int-ball2_simulator)

[![Ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-green.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ros2-humble installation](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

![Int-Ball2 Isaac Sim](img/int-ball2_isaac_sim.png)

## Table of Contents
1. [**Prerequisites**](#prerequisites)

2. [**Installation**](#installation)
    1. [Clone Repository](#clone-repository)
    2. [Install Dependencies](#install-dependencies)
    3. [Download Assets](#download-assets)

3. [**Usage**](#usage)
    1. [Build & Source](#build--source)
    2. [Launch the Simulation](#launch-the-simulation)
    3. [Teleoperation (Joy Controller)](#teleoperation-joy-controller)
    4. [Data Visulisation](#data-visualisation)

4. [**Data Visualisation**](#data-visualisation)

---

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
$ mkdir -p ~/int-ball2_ws/src
$ cd ~/int-ball2_ws/src
```

Clone the packages to your workspace.
```bash
$ git clone https://github.com/sd-robotics/int-ball2_isaac_sim.git
```

### Install Dependencies
Update the list of available packages (Isaac ROS).
```bash
$ wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
$ grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
$ echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
$ sudo apt-get update
```

Move into the workspace.
```bash
$ cd ~/int-ball2_ws/
```

Install the dependencies.
``` bash
$ rosdep install --from-paths src --ignore-src -r -y
```

### Download Assets
Run the following command to download the assets.
Move into the project.
```bash
$ cd ~/int-ball2_ws/src/int-ball2_isaac_sim
```

Download the assets.
```bash
$ bash install_local.sh
```

## Usage
### Build & Source
Build and Setup the package.
```bash
$ cd ~/int-ball2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```

### Launch the Simulation
Launch the simulation by ros2 launch.
```bash
$ ros2 launch int-ball2_isaac_sim int-ball2_isaac_sim.launch.py gui:="~/int-ball2_ws/src/int-ball2_isaac_sim/assets/KIBOU.usd"
```

> [!TIP]
> If you are using laptop to run Isaac Sim and suffering the problem of monitor freezing when Isaac Sim is launched, you might want to switch the system to use the NVIDIA GPU by the following command.
> ```bash
> $ sudo prime-select nvidia
> ```
>
> This will result in a better performance in graphic-intensive tasks. To check if your laptop has successfully switched to NVIDIA GPU, you can use the command
> ```bash
> $ prime-select query
> ```

### Teleoperation (Joy Controller)
Build and Setup the package.
Source.
```bash
$ cd ~/int-ball2_ws
$ source install/setup.bash
```

Make sure that you have joystick controller connected to the PC before running the command.
Then Launch the teleop.
```bash
$ ros2 launch int-ball2_control int-ball2_teleop.launch.py
```

Left stick for X-axis and Y-axis translational movement, B button + RT or LT for Z-axis translational movement.
Right stick for X-axis and Y-axis rotational movement, A button + RT or LT for Z-axis translational movement.
![Int-Ball2 Isaac Sim](img/int-ball2_teleop.png)

## Data Visualisation
Launch the Rviz.
```bash
$ ros2 launch int-ball2_control rviz_visualize.launch.py 
```

---

[Back to Top](#int-ball2-simulator)
