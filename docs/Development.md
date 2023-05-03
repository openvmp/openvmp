# Configuring the OpenVMP development environment

## Installing
Install ROS2 Humble, Gazebo, other development packages, and OpenVMP on an Ubuntu 22.04 system.

1. Follow the [ROS2 installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Choose the Desktop Install and also install the Development Tools.

2. As per the [Gazebo Humble instructions](https://gazebosim.org/docs/garden/ros_installation#ros-2-humble-and-ros-2-rolling), run this command: 

```bash
sudo apt install -y ros-humble-ros-gz
```

3. OpenVMP requires these additional packages:

```bash
sudo apt install -y git gnupg lsb-release
sudo apt install -y socat xterm
sudo apt install -y ansible
ansible-galaxy collection install community.general
sudo apt install v4l2loopback-utils
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-camera-calibration-parsers ros-humble-camera-info-manager
sudo apt install -y ros-humble-gazebo-dev ros-humble-gazebo-ros
```

4. Clone OpenVMP:

```bash
git clone --recurse-submodules https://github.com/openvmp/openvmp.git
```

## Building

Set the ROS2 underlay, cd into the OpenVMP overlay, and build:

```bash
source /opt/ros/humble/setup.bash  # or .zsh instead of .bash
cd openvmp/platform
COLCON_HOME=$(pwd) colcon build
source ./install/local_setup.bash  # or .zsh instead of .bash
```

## Testing

Start an RViz window with a single robot using dumb hardware stubs:

```bash
ros2 launch openvmp_robot robot.launch.py use_fake_hardware:=true
```

Click the yellow circle under the robot to select a manual control mode.
