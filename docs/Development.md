# OpenVMP

## Configuring the development environment

### Prerequisites

The following instructions are for Ubuntu 22.04. Please, adjust them based on the needs of your OS.

```
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y git curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt install -y ros-humble-desktop-full ros-dev-tools
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# The following packages are required for testing purposes only
# (during the regular build process though)
sudo apt install -y socat
```

### Visual Studio Code

Build the project at least once to have the support files generated.

## Building

In order to build and use all OpenVMP ROS2 packages, use the following commands:

```
source /opt/ros/humble/setup.bash  # or .zsh instead of .bash
git clone --recurse-submodules git://github.com/openvmp/openvmp.git
cd openvmp/platform
COLCON_HOME=$(pwd) colcon build
source ./install/local_setup.bash  # or .zsh instead of .bash
```

## Testing

In order to see how most of the software modules play together,
use the following command:

```
ros2 launch openvmp_robot robot.launch.py use_fake_hardware:=true
```

It will start an RViz window with a single robot using dumb hardware stubs.

See the robot id in the output of `ros2 node list` (4 alhanumeric characters). Using the robot id, send the following commands to control the robot:

```
ros2 run openvmp_motion_control_py stand --ros-args -p unit:=<robot id>
ros2 run openvmp_motion_control_py walk --ros-args -p unit:=<robot id>
```

![ROS/ROS2 index package for OpenVMP module: Development](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2FDevelopment.md&dt=OpenVMP%20Documentation)
