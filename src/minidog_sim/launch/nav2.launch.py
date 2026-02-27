from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    log_level = LaunchConfiguration("log_level")
    robot_type = LaunchConfiguration("robot_type")

    pkg_share = FindPackageShare("minidog_sim")

    # Select params and BT based on robot_type
    default_params = PathJoinSubstitution([
        pkg_share, "config",
        PythonExpression([
            "'nav2_diffbot.yaml' if '", robot_type, "' == 'diffbot' else 'nav2_slam.yaml'"
        ]),
    ])
    bt_xml = PathJoinSubstitution([
        pkg_share, "config",
        PythonExpression([
            "'nav2_bt_diffbot.xml' if '", robot_type, "' == 'diffbot' else 'nav2_bt_ackermann.xml'"
        ]),
    ])

    # Nav2 server nodes (start first)
    server_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="log",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_nav"), ("/odom", "/odom")],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="log",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="log",
            parameters=[
                nav2_params_file,
                {"use_sim_time": use_sim_time},
                {"default_nav_to_pose_bt_xml": bt_xml},
            ],
            # Ensure Nav2 never publishes directly to /cmd_vel (mux owns /cmd_vel).
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="log",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            # Recovery behaviors can publish cmd_vel; keep them on /cmd_vel_nav.
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="log",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]

    # Lifecycle manager: delay 5s to let server nodes fully initialize.
    # Without this delay, it races the servers and fails to configure them.
    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="log",
        parameters=[
            nav2_params_file,
            {
                "use_sim_time": use_sim_time,
                "bond_timeout": 10.0,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_type", default_value="diffbot"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_params),
            DeclareLaunchArgument("log_level", default_value="warn"),
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    *server_nodes,
                ]
            ),
            # Delay lifecycle manager to avoid racing server node initialization
            TimerAction(
                period=5.0,
                actions=[
                    GroupAction([
                        PushRosNamespace(namespace),
                        lifecycle_mgr,
                    ]),
                ],
            ),
        ]
    )
