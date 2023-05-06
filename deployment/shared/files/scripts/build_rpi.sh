#!/bin/bash

source /opt/ros/humble/setup.bash
#[ -f ./install/local_setup.bash ] && source ./install/local_setup.bash

# --parallel-workers 2: colcon spawns as many workers as there are CPUs, but
#                       CPU performance is not the bottleneck on Raspberry PI.
#                       SD card I/O is. Using two workers so that one is always
#                       waiting on I/O and the other one is busy preparing
#                       the next chunk of data for I/O.
#
# --packages-skip openvmp_hardware_simulation_gazebo: no gazebo dependencies
COLCON_HOME=$(pwd) colcon build \
  --parallel-workers 2 \
  --packages-skip openvmp_hardware_simulation_gazebo gazebo_ros2_control