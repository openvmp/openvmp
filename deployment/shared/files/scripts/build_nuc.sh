#!/bin/bash

source /opt/ros/humble/setup.bash
#[ -f ./install/local_setup.bash ] && source ./install/local_setup.bash

COLCON_HOME=$(pwd) colcon build \
  --packages-skip openvmp_hardware_simulation_gazebo gazebo_ros2_control