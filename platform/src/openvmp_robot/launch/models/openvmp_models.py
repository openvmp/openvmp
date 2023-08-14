import os
import tempfile

from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.utils import openvmp_utils
from openvmp_robot.launch.config import openvmp_config
from openvmp_robot.launch.config import files as openvmp_config_files


def launch_desc_deploy(context):
    # TODO(clairbee): implement this
    return []


# Spawn the robot in Gazebo
def launch_desc_spawn(context, robot_id, pos):
    namespace = openvmp_utils.generate_prefix(robot_id)
    robot_kind = openvmp_config.get_kind(context)

    x = 2.0 + 2.0 * (pos % 5)
    y = 2.0 * int(pos / 5)

    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        # name="gazebo_spawn_entity_" + robot_id,
        output="screen",
        namespace=namespace,
        arguments=[
            "-entity",
            "openvmp_robot_" + robot_kind + "_" + robot_id,
            "-file",
            openvmp_config_files.get_robot_description_file(
                context,
                robot_id,
                extra_xacro_params=[],
                # extra_xacro_params=[" mesh_extension_collision:=obj "],
            ),
            "-robot_namespace",
            namespace,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            "0.5",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "3.1415926",
        ],
    )

    return [
        start_gazebo_spawner_cmd,
    ]
