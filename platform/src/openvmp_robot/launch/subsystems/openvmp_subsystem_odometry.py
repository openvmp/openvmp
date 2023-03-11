import os
import tempfile

from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "odometry"
    ):
        return []

    is_simulation = LaunchConfiguration("is_simulation")

    package_name = openvmp_config.get_package(context)
    namespace = openvmp_config.get_namespace(context)

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    ekf_config_path = os.path.join(pkg_share, "config/ekf.yaml")
    ekf_config_patched = tempfile.NamedTemporaryFile(delete=False)
    data = ""
    with open(ekf_config_path, "r") as file:
        data = file.read()

    data = data.replace("%NAMESPACE%", namespace)
    ekf_config_patched.write(data.encode())
    ekf_config_patched_path = ekf_config_patched.name
    # print("Using EKF configuration file:")
    # print(ekf_config_patched_path)
    # print(data)
    ekf_config_patched.close()

    desc = []

    # Robot Localization
    ekf_node_cmd = Node(
            package="robot_localization",
            executable="ekf_node",
            # name="ekf_filter_node",
            output="screen",
            namespace=namespace,
            arguments=[
                ekf_config_patched_path,
            ],
            parameters=[
                # os.path.join(pkg_share, "config/ekf.yaml"),
                {
                    "use_sim_time": context.launch_configurations["is_simulation"]
                    == "true",
                },
            ],
            remappings=[
                ("/tf", namespace + "/tf"),
            ],
        )
    # TODO(clairbee): temporarily disabled until it's working
    #desc.append(ekf_node_cmd)

    return desc
