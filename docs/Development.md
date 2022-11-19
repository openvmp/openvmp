# OpenVMP

## Configuring the development environment

### Prerequisites

```
apt install git
apt install python3-colcon-common-extensions

apt install ros-humble-desktop-full
apt install ros-humble-ros2-control
apt install ros-humble-ros2-controllers
apt install ros-humble-gazebo-ros-pkgs
apt install ros-humble-gazebo-ros2-control

# The following packages are required for testing purposes only
# (during the regular build process though)
sudo apt install socat
```

### Visual Studio Code

Build the project at least once to have the support files generated.

## Building

In order to build and use all OpenVMP ROS2 packages, use the following commands:

```
git clone --recurse-submodules git://github.com/openvmp/openvmp.git
cd openvmp/platform
COLCON_HOME=$(pwd) colcon build
source ./install/local_setup.bash # or .zsh instead of .bash
```


![ROS/ROS2 index package for OpenVMP module: Development](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2FDevelopment.md&dt=OpenVMP%20Documentation)