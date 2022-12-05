import os
from openvmp_robot.launch.utils import openvmp_utils


def get_is_mac(context):
    return os.name == "Darwin"


def get_xacro_params(context, robot_id):
    simulate_str = "false"
    if context.launch_configurations["is_simulation"] == "true":
        simulate_str = "true"

    fake_hardware_str = "false"
    if context.launch_configurations["use_fake_hardware"] == "true":
        fake_hardware_str = "true"

    is_mac_str = "false"
    if get_is_mac(context):
        is_mac_str = "true"

    xacro_params = []
    xacro_params.append(" simulate:=" + simulate_str + " ")
    xacro_params.append(" fake_hardware:=" + fake_hardware_str + " ")
    xacro_params.append(" namespace:=" + openvmp_utils.generate_prefix(robot_id) + " ")
    xacro_params.append(" is_mac:=" + is_mac_str + " ")
    xacro_params.append(" package:=" + get_package(context) + " ")
    xacro_params.append(" robot_id:=" + robot_id + " ")
    return xacro_params


def get_kind(context):
    robot_kind = context.launch_configurations["kind"]
    return robot_kind


def get_robot_id(context):
    robot_id = context.launch_configurations["id"]
    return robot_id


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
