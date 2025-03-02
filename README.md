# Template Repository

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![日本語版 README](https://img.shields.io/badge/日本語-d9d9d9)](./README_JA.md)

![GitHub contributors](https://img.shields.io/github/contributors/SpaceData-Inc/temp_rep)
![GitHub issues](https://img.shields.io/github/issues/SpaceData-Inc/temp_rep)
![GitHub fork](https://img.shields.io/github/forks/SpaceData-Inc/temp_rep)
![GitHub stars](https://img.shields.io/github/stars/SpaceData-Inc/temp_rep)
[![Ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-green.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ros2-humble installation](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

![SD Robot](img/sd_robot.jpg)


## Table of Contents
1. [**Installation**](#installation)
    * [Local Installation](#local-installation)

2. [**Usage**](#usage)
    * [Build](#build--source)
    * [Launching Simulation](#launch-the-simulation-by-ros2-launch)
    * [Data Visulisation](#data-visualisation)



<!-- #### 参考
- [GitHub Flavored Markdown Spec](https://github.github.com/gfm/#introduction)
- [Markdown記法 チートシート](https://qiita.com/Qiita/items/c686397e4a0f4f11683d) -->


## Installation
### Local Installation
#### Clone Repository
```bash
# Make a workspace
mkdir -p intball_ws/src && cd intball_ws/src
cd ~/intball_ws/src

# Clone the packages to your workspace
git clone git@github.com:sd-robotics/int-ball2_simulator.git
```

#### Install Dependencies
```bash
cd ~/intball_ws/ && sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

#### Download Assets
Run the following command at the root of the repository to download the assets.
```bash
./install_local.sh
```

## Usage

### Build & Source

```bash
cd ~/intball_ws/
colcon build --symlink-install
source install/setup.bash
```

### Launch the Simulation by ros2 launch

```bash
ros2 launch kibou_isaac kibou_isaacsim.launch.py gui:="~/intball_ws/src/assets/KIBOU.usd"
```

> [!TIP]
> If you are using laptop to run IsaacSim and suffering the problem of monitor freezing when IsaacSim is launched, you might want to switch the system to use the NVIDIA GPU by the following command.
>
> ```bash
> sudo prime-select nvidia
> ```
>
> This will result in a better performance in graphic-intensive tasks. To check if your laptop has successfully switched to NVIDIA GPU, you can use the command
>
> ```bash
> prime-select query
> ```

### Teleoperation
```bash
ros2 launch intball_isaac intball_teleop.launch.py 
```

[Back to Top](#template-repository)

* Running the teleop control
> [!NOTE]
> Make sure that you have joystick controller connected to the PC.
```bash
ros2 launch intball_isaac intball_teleop.launch.py 
```

## Data Visualisation
#### Rviz
```bash
ros2 launch robot_vistool rviz_visualize.launch.py 
```

#### Foxglove Studio
```bash
ros2 launch robot_vistool foxglove_visualize.launch.py 
```
> [!TIP]
> If you don't have Foxglove studio, check [Foxglove installation](https://docs.foxglove.dev/docs/foxglove-agent/installation) adn [ROS2 bridge](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2)
> You can find the example layout inside `robot_vistool/foxglove/IntBall.json`
