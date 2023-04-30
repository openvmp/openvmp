import os
import subprocess
import time

from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config, files


def launch_desc(context):
    if (
        "subsystem" in context.launch_configurations
        and context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "dds"
    ):
        return []

    subprocess.run(["fastdds shm clean"], shell=True)
    time.sleep(3)

    xml_path = files.get_fastrtps_profile_server_file()
    print(xml_path)
    cmd_line = (
        f"FASTRTPS_DEFAULT_PROFILES_FILE={xml_path} fastdds discovery --server-id 0"
    )
    print(cmd_line)
    subprocess.Popen(cmd_line, shell=True)

    if "ip" in context.launch_configurations:
        robot_ip = context.launch_configurations["ip"]
    else:
        robot_ip = "127.0.0.1"

    return [
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
        SetEnvironmentVariable(
            name="FASTRTPS_DEFAULT_PROFILES_FILE",
            value=files.get_fastrtps_profile_client_file(context),
        ),
        SetEnvironmentVariable(
            name="ROS_DISCOVERY_SERVER",
            value=(robot_ip + ":11811"),
        ),
        # TODO(clairbee): use the below code instead of 'subprocess' above
        # ExecuteProcess(
        #     condition=LaunchConfigurationEquals("pos", "-1"),
        #     cmd=[
        #         "fastdds",
        #         "discovery",
        #         "--server-id",
        #         "0",
        #     ],
        #     # cmd=["fastdds discovery --server-id 0"],
        #     # cmd=["bash", "-c", "fastdds discovery --server-id 0"],
        #     additional_env={
        #         "FASTRTPS_DEFAULT_PROFILES_FILE": files.get_fastrtps_profile_server_file(),
        #     },
        #     output="screen",
        #     shell=True,
        # ),
    ]
