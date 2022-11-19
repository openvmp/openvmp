import os

from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.utils import openvmp_utils
from openvmp_robot.launch.config import openvmp_config


def launch_desc_deploy(context):
    # TODO(clairbee): implement this
    return []


# Spawn the robot in Gazebo
def launch_desc_spawn(context, robot_id, pos):

    namespace = openvmp_utils.generate_prefix(robot_id)
    robot_kind = openvmp_config.get_kind(context)
    package_name = openvmp_config.get_package(context)

    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # TODO(clairbee): refactor the model filename
    model_path = os.path.join(pkg_share, "models/" + package_name + ".urdf")

    # Launch robot_state_published
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", model_path]
                    + openvmp_config.get_xacro_params(context, robot_id, True)
                ),
                "use_sim_time": True,
            }
        ],
        arguments=[
            model_path,
            "--ros-args",
            "-r",
            "/tf:=" + namespace + "/tf",
            "-r",
            "/tf_static:=" + namespace + "/tf_static",
            # "--log-level",
            # "debug",
        ],
    )

    x = 2.0 * (pos % 5)
    y = 2.0 * int(pos / 5)

    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        # name="gazebo_spawn_entity",
        output="screen",
        namespace=namespace,
        arguments=[
            "-entity",
            "openvmp_robot_" + robot_kind + "_" + robot_id,
            "-topic",
            "robot_description",
            "-robot_namespace",
            namespace,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    return [
        start_robot_state_publisher_cmd,
        start_gazebo_spawner_cmd,
    ]
