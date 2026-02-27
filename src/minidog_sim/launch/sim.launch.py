from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    quiet_terminal = LaunchConfiguration("quiet_terminal")
    headless = LaunchConfiguration("headless")
    robot_type = LaunchConfiguration("robot_type")

    pkg_share = FindPackageShare("minidog_sim")

    # Select model based on robot_type
    diffbot_sdf_path = PathJoinSubstitution([pkg_share, "models", "diffbot", "model.sdf"])
    minidog_sdf_path = PathJoinSubstitution([pkg_share, "models", "minidog", "model.sdf"])

    # ----------------------------------------------------------------
    # Gazebo variants
    # ----------------------------------------------------------------
    gz_headless = ExecuteProcess(
        cmd=[
            "/usr/bin/ign", "gazebo",
            "-s",
            "-r", "-v", "0",
            world,
            "--force-version", "6",
        ],
        output="log",
        condition=IfCondition(headless),
    )

    _not_headless_quiet = PythonExpression([
        "'", headless, "' != 'true' and '", quiet_terminal, "' == 'true'"
    ])
    _not_headless_verbose = PythonExpression([
        "'", headless, "' != 'true' and '", quiet_terminal, "' != 'true'"
    ])

    gz_screen = ExecuteProcess(
        cmd=[
            "/usr/bin/ign", "gazebo",
            "-r", "-v", "2",
            world,
            "--render-engine-gui", "ogre",
            "--force-version", "6",
        ],
        output="screen",
        condition=IfCondition(_not_headless_verbose),
    )
    gz_log = ExecuteProcess(
        cmd=[
            "/usr/bin/ign", "gazebo",
            "-r", "-v", "0",
            world,
            "--render-engine-gui", "ogre",
            "--force-version", "6",
        ],
        output="log",
        condition=IfCondition(_not_headless_quiet),
    )

    # ----------------------------------------------------------------
    # Spawn robot — select model & name based on robot_type
    # ----------------------------------------------------------------
    _is_diffbot = PythonExpression(["'", robot_type, "' == 'diffbot'"])
    _is_ackermann = PythonExpression(["'", robot_type, "' != 'diffbot'"])

    spawn_diffbot = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-world", world_name,
            "-file", diffbot_sdf_path,
            "-name", "diffbot",
            "-allow_renaming", "true",
            "-x", "0.0", "-y", "0.0", "-z", "0.15",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_is_diffbot),
    )

    spawn_ackermann = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-world", world_name,
            "-file", minidog_sdf_path,
            "-name", "minidog",
            "-allow_renaming", "true",
            "-x", "0.0", "-y", "0.0", "-z", "0.15",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_is_ackermann),
    )

    spawn_after_gz = TimerAction(period=3.0, actions=[spawn_diffbot, spawn_ackermann])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("robot_type", default_value="diffbot"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [pkg_share, "worlds", "minidog_world.sdf"]
                ),
            ),
            DeclareLaunchArgument("world_name", default_value="minidog_world"),
            gz_headless,
            gz_screen,
            gz_log,
            spawn_after_gz,
        ]
    )
