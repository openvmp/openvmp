# OpenVMP

## Configuring the development environment

### OS

The following instructions are for Ubuntu 22.04.
If you would like to use another OS for development
then simply [use docker](../docker/dev/README.md).

### OS prerequisites

Use the following instructions to prepare the ROS2 environment on a vanilla Ubuntu 22.04 (not a docker container with ROS2 Humble preinstalled):

```bash
sudo -s
apt update
apt install -y software-properties-common
add-apt-repository universe
apt update
apt install -y git curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

The following OS packages are required for development and testing purposes:
```bash
apt install -y socat xterm

apt install -y ansible
ansible-galaxy collection install community.general
```

### ROS2 package prerequisites

The following packages might be missing from your ROS2 environment and
need to be installed additionally:

```bash
sudo -s
apt update
apt install -y ros-humble-desktop-full python3-colcon-common-extensions
apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
apt install -y ros-humble-camera-calibration-parsers v4l2loopback-utils
apt install -y ros-humble-topic-tools ros-humble-robot-localization
apt install -y ros-humble-ros-testing
```

### Visual Studio Code

Open the top level OpenVMP folder as a workspace in Visual Studio Code.

If you open any C++ file in Visual Studio Code
then there will be a large number of C++ errors reported.
The whole platform needs to be built at least once to have the neccessary
support files generated and to have those error messages go away.

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
sudo usermod -a -G video `whoami`  # Run this only once, may require re-login

./src/openvmp_robot_don1/scripts/fake_hardware_setup.sh
ros2 launch openvmp_robot robot.launch.py use_fake_hardware:=true
```

It will start an RViz window with a single robot using dumb hardware stubs.
Click on the yellow circle under the robot
to switch to any of the supported manual control modes.
