import os
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config
from openvmp_robot.launch.config import files as openvmp_config_files


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "motion_control"
    ):
        return []

    is_simulation = LaunchConfiguration("is_simulation")

    namespace = openvmp_config.get_namespace(context)

    controller_manager_cmd = Node(
        condition=UnlessCondition(is_simulation),
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "use_sim_time": context.launch_configurations["is_simulation"]
                == "true",
                "robot_description": openvmp_config_files.get_robot_description(
                    context
                ),
            },
            openvmp_config_files.get_ros2_controllers_path(context),
        ],
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    position_controller_spawner_cmd = Node(
        package="controller_manager",
        executable="spawner",
        # name="controller_spawner_position",
        output="screen",
        namespace=namespace,
        arguments=[
            "-c",
            namespace + "/controller_manager",
            "position_controller",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": context.launch_configurations["is_simulation"]
                == "true",
            }
        ],
    )

    velocity_controller_spawner_cmd = Node(
        package="controller_manager",
        executable="spawner",
        # name="controller_spawner_velocity",
        output="screen",
        namespace=namespace,
        arguments=[
            "-c",
            namespace + "/controller_manager",
            "velocity_controller",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": context.launch_configurations["is_simulation"]
                == "true",
            }
        ],
    )

    trajectory_controller_spawner_cmd = Node(
        package="controller_manager",
        executable="spawner",
        # name="controller_spawner_trajectory",
        output="screen",
        namespace=namespace,
        arguments=[
            "-c",
            namespace + "/controller_manager",
            "trajectory_controller",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": context.launch_configurations["is_simulation"]
                == "true",
            }
        ],
    )

    return [
        controller_manager_cmd,
        position_controller_spawner_cmd,
        velocity_controller_spawner_cmd,
        trajectory_controller_spawner_cmd,
    ]
