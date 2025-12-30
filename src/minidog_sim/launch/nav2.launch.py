from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("minidog_sim"), "config", "nav2_slam.yaml"]
    )

    nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_nav"), ("/odom", "/odom")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            # Ensure Nav2 never publishes directly to /cmd_vel (mux owns /cmd_vel).
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            # Recovery behaviors can publish cmd_vel; keep them on /cmd_vel_nav.
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("nav2_params_file", default_value=default_params),
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    *nodes,
                ]
            ),
        ]
    )


