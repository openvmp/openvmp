import sys

sys.path.append("src")

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from openvmp_robot.launch.utils import openvmp_utils
from openvmp_robot.launch.subsystems import *


def generate_launch_description_real():
    # kind = LaunchConfiguration("kind")
    declare_kind_cmd = DeclareLaunchArgument(
        name="kind",
        default_value="don1",
        description="The kind of the OpenVMP robots to spawn",
    )

    # id = LaunchConfiguration("id")
    declare_id_cmd = DeclareLaunchArgument(
        name="id",
        default_value=openvmp_utils.generate_id(),
        description="The id of the robot",
    )

    # subsystem = LaunchConfiguration("subsystem")
    declare_subsystem_cmd = DeclareLaunchArgument(
        name="subsystem",
        default_value="all",
        description="The subsystem to launch: 'all', 'reflection', 'drivers', 'motion_control', 'teleop', ...",
    )

    # is_simulation = LaunchConfiguration("is_simulation")
    declare_is_simulation_cmd = DeclareLaunchArgument(
        name="is_simulation",
        default_value="false",
        description="Apply tweaks required to operate in a simulated environment",
    )

    # use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        name="use_fake_hardware",
        default_value="false",
        description="Launch driver stubs that are not connected to real hardware",
    )

    # pos = LaunchConfiguration("pos")
    declare_pos_cmd = DeclareLaunchArgument(
        name="pos",
        default_value="-1",
        description="-1 for a real robots, or the index of simulated instance to help layout windows in a convinient way",
    )

    launch_desc = [
        declare_kind_cmd,
        declare_id_cmd,
        declare_subsystem_cmd,
        declare_is_simulation_cmd,
        declare_pos_cmd,
        OpaqueFunction(function=openvmp_subsystem_reflection.launch_desc),
        OpaqueFunction(function=openvmp_subsystem_drivers.launch_desc),
        # TODO(clairbee): subsystem: power_management
        # TODO(clairbee): subsystem: odometry
        # TODO(clairbee): subsystem: vision
        # TODO(clairbee): subsystem: local_slam
        # TODO(clairbee): subsystem: global_slam
        # TODO(clairbee): subsystem: perception
        # TODO(clairbee): subsystem: wan
        # TODO(clairbee): subsystem: lan
        # TODO(clairbee): subsystem: mission_control
        # TODO(clairbee): subsystem: task_control
        # TODO(clairbee): subsystem: motion_planning
        OpaqueFunction(function=openvmp_subsystem_motion_control.launch_desc),
        # TODO(clairbee): subsystem: navigation
        OpaqueFunction(function=openvmp_subsystem_teleop.launch_desc),
    ]

    return launch_desc


def generate_launch_description():
    return LaunchDescription(generate_launch_description_real())
