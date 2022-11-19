import os
from openvmp_robot.launch.utils import openvmp_utils


def get_is_mac(context):
    return os.name == "Darwin"


def get_xacro_params(context, robot_id, simulate=False):
    simulate_str = "false"
    if simulate:
        simulate_str = "true"
    xacro_params = []
    xacro_params.append(" simulate:=" + simulate_str + " ")
    xacro_params.append(" namespace:=" + openvmp_utils.generate_prefix(robot_id) + " ")
    xacro_params.append(" is_mac:=true ")
    xacro_params.append(" package:=" + get_package(context) + " ")
    xacro_params.append(" robot_id:=" + robot_id + " ")
    return xacro_params


def get_kind(context):
    robot_kind = context.launch_configurations["kind"]
    return robot_kind


def get_robot_id(context):
    robot_kind = context.launch_configurations["id"]
    return robot_kind


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
