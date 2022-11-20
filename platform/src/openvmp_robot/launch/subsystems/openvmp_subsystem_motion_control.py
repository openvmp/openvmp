from launch_ros.actions import Node


from openvmp_robot.launch.config import openvmp_config


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "motion_control"
    ):
        return []

    namespace = openvmp_config.get_namespace(context)

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
        # velocity_controller_spawner_cmd,
        trajectory_controller_spawner_cmd,
    ]
