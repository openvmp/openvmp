from launch.actions import TimerAction
from launch_ros.actions import Node

from openvmp_robot.launch.config import openvmp_config
from openvmp_robot.launch.config import files as openvmp_config_files


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "reflection"
    ):
        return []

    namespace = openvmp_config.get_namespace(context)
    robot_id = context.launch_configurations["id"]

    # Launch robot_state_published
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="robot_state_publisher_" + robot_id,
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "robot_description": openvmp_config_files.get_robot_description(
                    context, robot_id
                ),
                "use_sim_time": context.launch_configurations["is_simulation"]
                == "true",
            }
        ],
        arguments=[
            # model_path,
            "--ros-args",
            "-r",
            "/tf:=" + namespace + "/tf",
            "-r",
            "/tf_static:=" + namespace + "/tf_static",
            # "--log-level",
            # "debug",
        ],
    )

    joint_state_broadcaster_spawner_cmd = Node(
        package="controller_manager",
        executable="spawner",
        # name="controller_spawner_joint_state",
        output="screen",
        namespace=namespace,
        arguments=[
            "-c",
            namespace + "/controller_manager",
            "joint_state_broadcaster",
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
        start_robot_state_publisher_cmd,
        TimerAction(
            period=9.0,
            actions=[
                joint_state_broadcaster_spawner_cmd,
            ],
        ),
    ]
