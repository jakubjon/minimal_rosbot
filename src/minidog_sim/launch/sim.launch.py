from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    quiet_terminal = LaunchConfiguration("quiet_terminal")

    robot_sdf_path = PathJoinSubstitution(
        [FindPackageShare("minidog_sim"), "models", "minidog", "model.sdf"]
    )

    # Run Gazebo directly so we can control output routing (quiet vs noisy).
    # NOTE: On WSLg, Ogre2 (default) often crashes. Ogre1 is stable.
    gz_cmd_verbose = [
        "/usr/bin/ign",
        "gazebo",
        "-r",
        "-v",
        "2",
        world,
        "--render-engine-gui",
        "ogre",
        "--force-version",
        "6",
    ]
    gz_cmd_quiet = [
        "/usr/bin/ign",
        "gazebo",
        "-r",
        "-v",
        "0",
        world,
        "--render-engine-gui",
        "ogre",
        "--force-version",
        "6",
    ]

    gz_screen = ExecuteProcess(
        cmd=gz_cmd_verbose,
        output="screen",
        condition=UnlessCondition(quiet_terminal),
    )
    gz_log = ExecuteProcess(
        cmd=gz_cmd_quiet,
        output="log",
        condition=IfCondition(quiet_terminal),
    )

    spawn_screen = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            world_name,
            "-file",
            robot_sdf_path,
            "-name",
            "minidog",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.15",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(quiet_terminal),
    )
    spawn_log = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-world",
            world_name,
            "-file",
            robot_sdf_path,
            "-name",
            "minidog",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.15",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(quiet_terminal),
    )

    # Give Gazebo a moment to initialize the world before spawning.
    spawn_after_gz = TimerAction(period=2.0, actions=[spawn_screen, spawn_log])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
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
            gz_screen,
            gz_log,
            spawn_after_gz,
        ]
    )


