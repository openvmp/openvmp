import os
import sys

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

# TODO:
# - use src/openvmp_robot/config/simulation.yaml to propagate use_sim_time or delete that file
# - cleanup is_mac and add support for Windows


def spawn_func(ctx, rid, pos):
    return openvmp_models.launch_desc_spawn(ctx, rid, pos)


def include_unit_launch_descriptions(context):
    launch_desc = []

    pkg_share = FindPackageShare(package="openvmp_robot").find("openvmp_robot")
    robot_launch_py_path = os.path.join(pkg_share, "launch/robot.launch.py")

    for pos in range(int(context.launch_configurations["num"])):
        robot_id = openvmp_utils.generate_id()

        launch_desc.extend(
            [
                OpaqueFunction(function=spawn_func, args=[robot_id, pos]),
                TimerAction(
                    period=5.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(robot_launch_py_path),
                            launch_arguments={
                                "kind": context.launch_configurations["kind"],
                                "id": robot_id,
                                "is_simulation": "true",
                                "pos": str(pos),
                            }.items(),
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

    launch_desc = [
        declare_kind_cmd,
        declare_num_cmd,
        OpaqueFunction(function=openvmp_worlds.launch_desc),
        OpaqueFunction(function=openvmp_models.launch_desc_deploy),
        TimerAction(
            period=20.0,
            actions=[
                OpaqueFunction(function=include_unit_launch_descriptions),
            ],
        ),
    ]

    return LaunchDescription(launch_desc)
