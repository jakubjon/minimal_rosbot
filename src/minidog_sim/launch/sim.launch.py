from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")

    robot_sdf_path = PathJoinSubstitution(
        [FindPackageShare("minidog_sim"), "models", "minidog", "model.sdf"]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            # Fortress / ign-gazebo6
            "gz_version": "6",
            # NOTE: On WSLg, Ogre2 (default) often crashes. Ogre1 is stable.
            "gz_args": ["-r -v 2 ", world, " --render-engine-gui ogre"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn = Node(
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
    )

    # Give Gazebo a moment to initialize the world before spawning.
    spawn_after_gz = TimerAction(period=2.0, actions=[spawn])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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
            gz_sim_launch,
            spawn_after_gz,
        ]
    )


