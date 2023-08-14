import os
import sys
import time

sys.path.append("src")

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config
from openvmp_robot.launch.utils import openvmp_utils
from openvmp_robot.launch.worlds import openvmp_worlds
from openvmp_robot.launch.models import openvmp_models
from openvmp_robot.launch.subsystems import openvmp_subsystem_dds

# TODO:
# - use src/openvmp_robot/config/simulation.yaml to propagate use_sim_time or delete that file
# - cleanup is_mac and add support for Windows


def spawn_func(ctx, rid, pos):
    return openvmp_models.launch_desc_spawn(ctx, rid, pos)


def include_unit_launch_descriptions(context):
    launch_desc = []

    pkg_share = FindPackageShare(package="openvmp_robot").find("openvmp_robot")
    robot_launch_py_path = os.path.join(pkg_share, "launch/robot.launch.py")

    pos_limit = int(context.launch_configurations["num"])
    for pos in range(pos_limit):
        robot_id = openvmp_utils.generate_id()

        if pos == 0 and pos_limit == 1:
            pos_to_pass = -1
        else:
            pos_to_pass = pos

        launch_desc.extend(
            [
                TimerAction(
                    period=0.0 + 7.0 * pos,
                    actions=[
                        OpaqueFunction(function=spawn_func, args=[str(robot_id), pos]),
                        TimerAction(
                            period=7.0 + 7.0 * pos,
                            actions=[
                                IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(robot_launch_py_path),
                                    launch_arguments={
                                        "kind": context.launch_configurations["kind"],
                                        "id": robot_id,
                                        "is_simulation": "true",
                                        "simulate_remote_hardware_interface": context.launch_configurations[
                                            "simulate_remote_hardware_interface"
                                        ],
                                        "pos": str(pos_to_pass),
                                    }.items(),
                                ),
                            ],
                        ),
                    ],
                ),
            ]
        )

    return launch_desc


def generate_launch_description():
    declare_kind_cmd = DeclareLaunchArgument(
        name="kind",
        default_value="don1",
        description="The kind of the OpenVMP robots to spawn",
    )
    declare_num_cmd = DeclareLaunchArgument(
        name="num",
        default_value="1",
        description="The number of the robots to spawn",
    )

    # simulate_remote_hardware_interface = LaunchConfiguration("simulate_remote_hardware_interface")
    declare_simulate_remote_hardware_interface_cmd = DeclareLaunchArgument(
        name="simulate_remote_hardware_interface",
        default_value="false",
        description="Do not use Gazebo plugin for ros2_control. Use the same hardware interface as the real robots.",
    )

    pkg_share = FindPackageShare(package="openvmp_robot_don1").find(
        "openvmp_robot_don1"
    )
    use_meshes_default = "none"
    if os.path.exists(pkg_share + "/meshes/hip.stl"):
        use_meshes_default = "low"
    declare_use_meshes_cmd = DeclareLaunchArgument(
        name="use_meshes",
        default_value=use_meshes_default,
        description="Use meshes for visualization (none/low/high)",
    )

    launch_desc = [
        declare_kind_cmd,
        declare_num_cmd,
        declare_simulate_remote_hardware_interface_cmd,
        declare_use_meshes_cmd,
        OpaqueFunction(function=openvmp_subsystem_dds.launch_desc),
        OpaqueFunction(function=openvmp_worlds.launch_desc),
        OpaqueFunction(function=openvmp_models.launch_desc_deploy),
        TimerAction(
            period=15.0,
            actions=[
                OpaqueFunction(function=include_unit_launch_descriptions),
            ],
        ),
    ]

    return LaunchDescription(launch_desc)
