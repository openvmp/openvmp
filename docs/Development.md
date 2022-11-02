# OpenVMP

## Configuring the development environment

### Prerequisites

```
apt install git
apt install python3-colcon-common-extensions

apt install ros-humble-desktop-full
apt install ros-humble-gazebo-ros-pkgs
apt install ros-humble-ros2-control

```

### Visual Studio Code

Build the project at least once to have the support files generated.

## Building

### ROS2

In order to build and use all OpenVMP ROS2 packages, use the following commands:

```
git submodule update --init --recursive
cd platform
COLCON_HOME=$(pwd) colcon build
source ./install/local_setup.bash # or .zsh
```

## Testing

### Testing prerequisites

The following tools need to be pre-installed on your OS:

- socat

### Simulation

```
ros2 launch openvmp_robot_don1 openvmp_robot_don1.launch.py simulate:=true
```

Please, note, the first simulation run will take time to start.


![ROS/ROS2 index package for OpenVMP module: Development](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2FDevelopment.md&dt=OpenVMP%20Documentation)