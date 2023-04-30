import os

from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.utils import openvmp_utils


def get_kind(context):
    robot_kind = context.launch_configurations["kind"]
    return robot_kind


def get_robot_id(context):
    robot_id = context.launch_configurations["id"]
    return robot_id


def get_robot_ip(context):
    if "ip" in context.launch_configurations:
        robot_ip = context.launch_configurations["ip"]
    else:
        robot_ip = "127.0.0.1"
    return robot_ip


def get_package(context):
    robot_kind = context.launch_configurations["kind"]
    return "openvmp_robot_" + robot_kind


def get_namespace(context):
    robot_id = context.launch_configurations["id"]
    return openvmp_utils.generate_prefix(robot_id)


def get_name(context):
    robot_kind = context.launch_configurations["kind"]
    robot_id = context.launch_configurations["id"]
    return "openvmp_robot_" + robot_kind + "_" + robot_id


def get_xacro_params(context, robot_id):
    package_name = get_package(context)
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    xacro_params = []
    xacro_params.append(" package:=" + get_package(context) + " ")
    xacro_params.append(" robot_id:=" + robot_id + " ")
    xacro_params.append(" namespace:=" + openvmp_utils.generate_prefix(robot_id) + " ")

    # Simulation params
    if (
        # simulation.world.launch.py does NO have is_simulation set
        not "is_simulation" in context.launch_configurations
        or context.launch_configurations["is_simulation"] == "true"
    ):
        xacro_params.append(" simulate:=true ")
        xacro_params.append(" has_extra_parameters_file:=true ")
        xacro_params.append(
            " extra_parameters_file:='$(find openvmp_robot)/config/simulation.yaml' "
        )
        # Simulate remote_hardware_interface
        if (
            "simulate_remote_hardware_interface" in context.launch_configurations
            and context.launch_configurations["simulate_remote_hardware_interface"] == "true"
        ):
            xacro_params.append(" simulate_remote:=true ")

    # Fake hardware params
    if (
        # simulation.world.launch.py does NO have is_simulation set
        "use_fake_hardware" in context.launch_configurations
        and context.launch_configurations["use_fake_hardware"] == "true"
    ):
        xacro_params.append(" fake_hardware:=true ")

    # OS params
    if os.name == "Darwin":
        xacro_params.append(" is_mac:=true ")
    else:
        xacro_params.append(" is_mac:=false ")

    return xacro_params
