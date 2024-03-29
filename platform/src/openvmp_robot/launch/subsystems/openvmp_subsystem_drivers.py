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

    desc = []
    if context.launch_configurations["is_simulation"]:
        # Interactive markers for Rviz
        desc.append(
            Node(
                condition=UnlessCondition(is_simulation),
                package="openvmp_hardware_manager",
                executable="openvmp_hardware_manager",
                # name="hardware_manager",
                output="screen",
                namespace=namespace,
                parameters=[
                    {
                        "config_path": hardware_config_path,
                        "use_fake_hardware": context.launch_configurations[
                            "use_fake_hardware"
                        ]
                        == "true",
                    },
                ],
                # arguments=[
                #     "--ros-args",
                #     "--log-level",
                #     "debug",
                # ],
                # prefix=["xterm -e gdb -ex run --args"],
            )
        )

    return desc
