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

    robot_sdf_path = PathJoinSubstitution(
        [FindPackageShare("minidog_sim"), "models", "minidog", "model.sdf"]
    )

    # ----------------------------------------------------------------
    # Gazebo variants
    # ----------------------------------------------------------------
    # Headless (server-only, no GUI) — saves ~60% CPU on WSL2.
    gz_headless = ExecuteProcess(
        cmd=[
            "/usr/bin/ign", "gazebo",
            "-s",  # server-only
            "-r", "-v", "0",
            world,
            "--force-version", "6",
        ],
        output="log",
        condition=IfCondition(headless),
    )

    # GUI mode (non-headless) — verbose or quiet
    _not_headless = PythonExpression(["'", headless, "' != 'true'"])
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
    # Spawn robot
    # ----------------------------------------------------------------
    spawn_args = [
        "-world", world_name,
        "-file", robot_sdf_path,
        "-name", "minidog",
        "-allow_renaming", "true",
        "-x", "0.0", "-y", "0.0", "-z", "0.15",
    ]

    spawn_screen = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=spawn_args,
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(quiet_terminal),
    )
    spawn_log = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=spawn_args,
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(quiet_terminal),
    )

    # Give Gazebo a moment to initialize the world before spawning.
    spawn_after_gz = TimerAction(period=3.0, actions=[spawn_screen, spawn_log])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("minidog_sim"),
                        "worlds",
                        "minidog_world.sdf",
                    ]
                ),
            ),
            DeclareLaunchArgument("world_name", default_value="minidog_world"),
            gz_headless,
            gz_screen,
            gz_log,
            spawn_after_gz,
        ]
    )
