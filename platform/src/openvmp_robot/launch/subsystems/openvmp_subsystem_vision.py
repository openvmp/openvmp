import os

from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config

FRAME_IDS = [
    "front_left_arm_camera_0",
    "front_left_arm_camera_1",
    "front_right_arm_camera_0",
    "front_right_arm_camera_1",
    "rear_left_arm_camera_0",
    "rear_left_arm_camera_1",
    "rear_right_arm_camera_0",
    "rear_right_arm_camera_1",
]

NODE_NAMES = [
    "camera_front_left_arm_0",
    "camera_front_left_arm_1",
    "camera_front_right_arm_0",
    "camera_front_right_arm_1",
    "camera_rear_left_arm_0",
    "camera_rear_left_arm_1",
    "camera_rear_right_arm_0",
    "camera_rear_right_arm_1",
]


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "vision"
    ):
        return []

    is_simulation = LaunchConfiguration("is_simulation")
    use_fake_hardware = LaunchConfiguration("is_simulation")

    package_name = openvmp_config.get_package(context)
    namespace = openvmp_config.get_namespace(context)

    desc = []
    if context.launch_configurations["is_simulation"]:
        for camera_id in range(8):
            if context.launch_configurations["use_fake_hardware"]:
                camera_calibration_url = (
                    "package://openvmp_robot/config/fake_camera.ini"
                )
            else:
                camera_calibration_url = (
                    "package://"
                    + package_name
                    + "config/camera_"
                    + str(camera_id)
                    + ".yaml"
                )
            usb_camera_driver_cmd = Node(
                condition=UnlessCondition(is_simulation),
                package="usb_camera_driver",
                executable="usb_camera_driver_node",
                name=NODE_NAMES[camera_id],
                output="screen",
                namespace=namespace + "/" + NODE_NAMES[camera_id],
                parameters=[
                    {
                        "frame_id": FRAME_IDS[camera_id],
                        "camera_id": camera_id,
                        "fps": 25.0,
                        "image_width": 320,
                        "image_height": 200,
                        "camera_calibration_file": camera_calibration_url,
                    },
                ],
                arguments=[
                    "--ros-args",
                    "-r",
                    "image:=image_raw",
                ],
            )
            # TODO(clairbee): temporarily disabled until it's useful
            #desc.append(usb_camera_driver_cmd)

    return desc
