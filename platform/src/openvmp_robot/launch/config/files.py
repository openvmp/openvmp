import os
import tempfile

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

from openvmp_robot.launch.config import openvmp_config
from openvmp_robot.launch.utils import openvmp_utils


def get_ros2_controllers_path(context, robot_id=None):
    # robot_id can be passed as a parameter if it is used outside of the context
    # specific to a particular robot (e.g. when initializing ros2_control in Gazebo)
    if robot_id is None:
        robot_id = openvmp_config.get_robot_id(context)
    namespace = openvmp_utils.generate_prefix(robot_id)
    package_name = openvmp_config.get_package(context)
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    controllers_config_path = os.path.join(pkg_share, "config/ros2_controllers.yaml")
    controllers_config_patched = tempfile.NamedTemporaryFile(delete=False)
    data = ""
    with open(controllers_config_path, "r") as file:
        data = file.read()

    data = data.replace("%NAMESPACE%", namespace)
    if (
        "is_simulation" in context.launch_configurations
        and context.launch_configurations["is_simulation"] == "true"
    ):
        data = data.replace("%USE_SIM_TIME%", "true")
    else:
        data = data.replace("%USE_SIM_TIME%", "false")

    controllers_config_patched.write(data.encode())
    controllers_config_patched_path = controllers_config_patched.name
    controllers_config_patched.close()

    return controllers_config_patched_path


def get_robot_description(context, robot_id=None):
    # robot_id can be passed as a parameter if it is used outside of the context
    # specific to a particular robot (e.g. when initializing ros2_control in Gazebo)
    if robot_id is None:
        robot_id = openvmp_config.get_robot_id(context)
    package_name = openvmp_config.get_package(context)
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # TODO(clairbee): refactor the model filename
    model_path = os.path.join(pkg_share, "models/" + package_name + ".urdf")

    xacro_params = openvmp_config.get_xacro_params(context, robot_id)
    xacro_params += [
        " controllers_yaml_path:=" + get_ros2_controllers_path(context, robot_id) + " "
    ]
    return Command(["xacro ", model_path] + xacro_params)


def get_robot_description_file(context, robot_id=None):
    subs = get_robot_description(context, robot_id)
    content = subs.perform(context)
    robot_description_file = tempfile.NamedTemporaryFile(delete=False)
    robot_description_file.write(str(content).encode())
    robot_description_file_path = robot_description_file.name
    robot_description_file.close()
    return robot_description_file_path
