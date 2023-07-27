import os
import tempfile

from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from openvmp_robot.launch.config import openvmp_config


def launch_desc(context):
    if (
        context.launch_configurations["subsystem"] != "all"
        and context.launch_configurations["subsystem"] != "teleop"
    ):
        return []

    package_name = openvmp_config.get_package(context)
    namespace = openvmp_config.get_namespace(context)

    # Interactive markers for Rviz
    start_control_interactive_cmd = Node(
        package="openvmp_control_interactive",
        executable="openvmp_control_interactive",
        # name="openvmp_control_interactive",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "use_sim_time": context.launch_configurations["is_simulation"]
                == "true",
            }
        ],
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    # Launch RViz
    pos = -1
    try:
        pos = int(context.launch_configurations["pos"])
    except:
        ignore_ = 1

    # TODO(clairbee): autodetect the screen layout and
    #                 guess convinient window locations

    # Horizontally, place RViz to the right from Gazebo.
    window_x = 1536
    # Vertically, place RViz at the top of the screen.
    window_y = 25
    # RViz consumes the rest of the screen
    window_width = 1536
    window_height = 1895
    if pos != -1:
        # Place RViz windows in two columns
        window_x = 1536 + 768 * int(pos / 2)
        # Place RViz windows in two rows
        window_y = 25 + 947 * (pos % 2)
        window_width = 768
        window_height = 947

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    rviz_config_path = os.path.join(pkg_share, "config/rviz.config")
    rviz_config_patched = tempfile.NamedTemporaryFile(delete=False)
    data = ""
    with open(rviz_config_path, "r") as file:
        data = file.read()

    data = data.replace("%NAMESPACE%", namespace)
    data = data.replace("%WINDOW_X%", str(window_x))
    data = data.replace("%WINDOW_Y%", str(window_y))
    data = data.replace("%WINDOW_WIDTH%", str(window_width))
    data = data.replace("%WINDOW_HEIGHT%", str(window_height))
    rviz_config_patched.write(data.encode())
    rviz_config_patched_path = rviz_config_patched.name
    # print("Using RViz2 configuration file:")
    # print(rviz_config_patched_path)
    # print(data)
    rviz_config_patched.close()

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        # name="rviz2",
        output="screen",
        namespace=namespace,
        arguments=[
            "-d",
            rviz_config_patched_path,
            "--ros-args",
            "-r",
            "/tf:=" + namespace + "/tf",
            "-r",
            "/tf_static:=" + namespace + "/tf_static",
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
        TimerAction(
            period=10.0,
            actions=[
                start_control_interactive_cmd,
            ],
        ),
        TimerAction(
            period=11.0,
            actions=[
                start_rviz_cmd,
            ],
        ),
    ]
