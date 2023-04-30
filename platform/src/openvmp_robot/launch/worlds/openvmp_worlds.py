from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from openvmp_robot.launch.config import openvmp_config


def launch_desc(context):
    package_name = openvmp_config.get_package(context)

    model_path = FindPackageShare(package=package_name).perform(context) + "/.."

    world = LaunchConfiguration("world")

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value="horizontal",
        description="The name of the world to launch (see 'src/openvmp_robot/worlds')",
    )

    try:
        home = Path.home()
        with open(str(home) + "/.gazebo/gui.ini", "w") as file:
            # The below assumes that there is one or two (ideally positioned)
            # 4K screens above MacBook Pro 16"
            file.write(
                """[geometry]
x=0
y=25
width=1536
height=2160
"""
                # x=-"""
                #                 + str(int((3840 - 3072) / 2))
                #                 + """
                # y=-2160
                # width=1920
                # height=2160
            )
            file.close()
    except Exception as e:
        print(e)
        ignore_ = 1

    start_gazebo_cmd = GroupAction(
        [
            PushRosNamespace("gazebo"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("gazebo_ros"),
                        "/launch/gazebo.launch.py",
                    ]
                ),
                launch_arguments={
                    "world": ["src/openvmp_robot/worlds/", world, ".world"],
                    "verbose": "true",
                    "lockstep": "true",
                    # "gdb": "true",
                }.items(),
            ),
        ]
    )

    return [
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=model_path),
        declare_world_cmd,
        start_gazebo_cmd,
    ]
