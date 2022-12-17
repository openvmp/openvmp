import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackage
from launch.substitutions import FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# TODO:
# - use src/openvmp_robot/config/simulation.yaml to propagate use_sim_time or delete that file
# - cleanup is_mac and add support for Windows

is_mac = False
if os.name == "Darwin":
    is_mac = True


def generate_launch_description():
    xacro_params = []
    if is_mac:
        xacro_params.append(" is_mac:=true ")

    # Set the path to different files and folders.
    robot_name = "openvmp_robot_don1"
    pkg_share = FindPackageShare(package=robot_name).find(robot_name)
    default_model_path = os.path.join(pkg_share, "models/openvmp_robot_don1.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "config/rviz.config")
    controllers_file = os.path.join(pkg_share, "config/ros2_controllers.yaml")

    # Launch configuration variables specific to simulation
    model = LaunchConfiguration("model")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    # use_ros2_control = LaunchConfiguration("use_ros2_control")
    # use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_gazebo = LaunchConfiguration("use_gazebo")

    # Declare the launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=default_rviz_config_path,
        description="Full path to the RVIZ config file to use",
    )

    # declare_use_ros2_control_cmd = DeclareLaunchArgument(
    #     name="use_ros2_control",
    #     default_value="False",
    #     description="Flag to disable ros2_control",
    # )

    declare_use_joint_state_publisher_gui_cmd = DeclareLaunchArgument(
        name="use_joint_state_publisher_gui",
        default_value="False",
        description="Flag to enable joint_state_publisher_gui",
    )

    # Always needed, make it unconfigurable for now
    # declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    #     name="use_robot_state_pub",
    #     default_value="True",
    #     description="Whether to start the robot state publisher",
    # )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name="use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name="use_gazebo",
        default_value="True",
        description="Use simulation (Gazebo)",
    )

    # Specify the actions

    # Package and deploy models
    package_sdf = ExecuteProcess(
        name="package_sdf",
        cmd=[
            [
                FindExecutable(name="python3"),
                " ",
                "src/openvmp_robot/scripts/package_sdf.py",
                # FindPackage(package="openvmp_robot")..find(
                #     "/lib/openvmp_robot/package_sdf.py"
                # ),
            ]
        ],
        shell=True,
    )

    # Launch cartographer
    start_cartographer_occupancy_grid_cmd = Node(
        # condition=UnlessCondition(cartographer),
        package="cartographer_ros",
        executable="occupancy_grid_node",
        parameters=[{"resolution": 0.01, "publish_period_sec": 1.0}],
    )

    start_cartographer_cmd = Node(
        # condition=UnlessCondition(cartographer),
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[
            {
                "configuration_directory": "install/openvmp_robot_don1/share/openvmp_robot_don1/config",
                "configuration_basename": "cartographer.lua",
            }
        ],
    )

    # Publish the joint state values for the non-fixed joints in the URDF file.
    # start_joint_state_publisher_cmd = Node(
    #     # condition=UnlessCondition(gui),
    #     package="openvmp_joint_state_publisher",
    #     executable="publisher",
    #     name="main",
    # )

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_state_publisher_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        # name="joint_state_publisher_gui",
    )

    controller_manager_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        condition=UnlessCondition(use_gazebo),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "use_sim_time": use_gazebo,
                "robot_description": Command(["xacro ", model] + xacro_params),
            },
            controllers_file,
        ],
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
        output="screen",
        # arguments=[default_model_path],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    position_controller_spawner_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        # condition=UnlessCondition(use_gazebo),
        package="controller_manager",
        executable="spawner",
        name="position_controller",
        arguments=[
            "position_controller",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": use_gazebo,
            }
        ],
        output="screen",
    )
    effort_controller_spawner_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        # condition=UnlessCondition(use_gazebo),
        package="controller_manager",
        executable="spawner",
        name="effort_controller",
        arguments=[
            "effort_controller",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": use_gazebo,
            }
        ],
        output="screen",
    )
    velocity_controller_spawner_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        # condition=UnlessCondition(use_gazebo),
        package="controller_manager",
        executable="spawner",
        name="velocity_controller",
        arguments=[
            "velocity_controller",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": use_gazebo,
            }
        ],
        output="screen",
    )
    trajectory_controller_spawner_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        # condition=UnlessCondition(use_gazebo),
        package="controller_manager",
        executable="spawner",
        name="trajectory_controller",
        arguments=[
            "trajectory_controller",
            "--ros-args",
            "--log-level",
            "debug",
        ],
        parameters=[
            {
                # "use_sim_time": use_gazebo,
            }
        ],
        output="screen",
    )
    joint_state_broadcaster_spawner_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        # condition=UnlessCondition(use_gazebo),
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            # "--ros-args",
            # "--log-level",
            # "debug",
        ],
        parameters=[
            {
                "use_sim_time": use_gazebo,
            }
        ],
        output="screen",
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = [
        # TODO(clairbee): keep a single 'cmd' and parametrize it
        Node(
            condition=IfCondition(use_gazebo),
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": Command(
                        ["xacro ", model] + xacro_params + [" simulate:=true"]
                    ),
                }
            ],
            arguments=[
                default_model_path
            ],  # TODO: consider adding 'use_gazebo' here and concatenate it with 'simulate:=true'
        ),
        Node(
            condition=UnlessCondition(use_gazebo),
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": use_gazebo,
                    "robot_description": Command(["xacro ", model] + xacro_params),
                }
            ],
            arguments=[default_model_path],
        ),
    ]

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        # name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {
                "use_sim_time": use_gazebo,
            }
        ],
    )

    # Interactive markers for Rviz
    start_control_interactive_cmd = Node(
        condition=IfCondition(use_rviz),
        package="openvmp_control_interactive",
        executable="openvmp_control_interactive",
        parameters=[
            {
                "use_sim_time": use_gazebo,
            }
        ],
        output="screen",
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={"world": "src/openvmp_robot/worlds/test1.world"}.items(),
    )

    # Spawn the robot in Gazebo
    start_gazebo_spawner_cmd = Node(
        condition=IfCondition(use_gazebo),
        package="gazebo_ros",
        executable="spawn_entity.py",
        # name="gazebo_spawn_entity",
        output="screen",
        arguments=[
            "-entity",
            "openvmp_robot_don1",
            "-topic",
            "robot_description",
            # "-database",
            # "openvmp_don1",
        ],
    )

    # Create the launch description and populate
    return LaunchDescription(
        [
            # Parameters
            declare_model_path_cmd,
            # declare_use_ros2_control_cmd,
            declare_use_joint_state_publisher_gui_cmd,
            # declare_use_robot_state_pub_cmd,
            declare_use_rviz_cmd,
            declare_rviz_config_file_cmd,
            declare_use_gazebo_cmd,
            # Actions
            package_sdf,
            # start_joint_state_publisher_cmd,
            start_joint_state_publisher_gui_node,
            controller_manager_cmd,
            start_robot_state_publisher_cmd[0],
            start_robot_state_publisher_cmd[1],
            # start_rviz_cmd,
            start_gazebo_cmd,
            # start_gazebo_spawner_cmd,
            TimerAction(
                period=20.0,
                actions=[
                    start_gazebo_spawner_cmd,
                ],
            ),
            # RegisterEventHandler(
            #     OnProcessStart(
            #         target_action=start_gazebo_cmd,
            #         on_start=[
            #             start_gazebo_spawner_cmd,
            #         ],
            #     )
            # ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=start_gazebo_spawner_cmd,
                    on_exit=[
                        joint_state_broadcaster_spawner_cmd,
                        trajectory_controller_spawner_cmd,
                        # TimerAction(
                        #     period=7.0,
                        #     actions=[
                        #     ],
                        # )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=joint_state_broadcaster_spawner_cmd,
                    on_start=[
                        start_control_interactive_cmd,
                        TimerAction(
                            period=1.0,
                            actions=[
                                start_rviz_cmd,
                            ],
                        ),
                    ],
                )
            ),
            # RegisterEventHandler(
            #     OnProcessStart(
            #         target_action=start_rviz_cmd,
            #         on_start=[
            #         ],
            #     )
            # ),
            # RegisterEventHandler(
            #     OnProcessStart(
            #         target_action=start_rviz_cmd,
            #         on_start=[
            #             TimerAction(
            #                 period=7.0,
            #                 actions=[
            #                     # position_controller_spawner_cmd,
            #                     # effort_controller_spawner_cmd,
            #                     # velocity_controller_spawner_cmd,
            #                     trajectory_controller_spawner_cmd,
            #                 ],
            #             )
            #         ],
            #     )
            # ),
        ]
    )
