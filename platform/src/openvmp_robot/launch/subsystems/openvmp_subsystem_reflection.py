from launch_ros.actions import Node


from openvmp_robot.launch.config import openvmp_config


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "reflection"
    ):
        return []

    namespace = openvmp_config.get_namespace(context)

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
        joint_state_broadcaster_spawner_cmd,
    ]
