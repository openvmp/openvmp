import os

from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "drivers"
    ):
        return []

    is_simulation = LaunchConfiguration("is_simulation")

    package_name = openvmp_config.get_package(context)
    namespace = openvmp_config.get_namespace(context)

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    hardware_config_path = os.path.join(pkg_share, "config/hardware.yaml")

    # Interactive markers for Rviz
    start_hardware_manager_cmd = Node(
        condition=UnlessCondition(is_simulation),
        package="openvmp_hardware_manager",
        executable="openvmp_hardware_manager",
        # name="hardware_manager",
        output="screen",
        namespace=namespace,
        arguments=[
            hardware_config_path,
        ],
    )

    return [
        start_hardware_manager_cmd,
    ]
