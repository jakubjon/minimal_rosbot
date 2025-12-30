from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("minidog_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    enable_slam = LaunchConfiguration("enable_slam")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    rmw_implementation = LaunchConfiguration("rmw_implementation")
    fastdds_profiles_file = PathJoinSubstitution([pkg_share, "config", "fastdds_no_shm.xml"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "sim.launch.py"])),
        launch_arguments={"world_name": world_name, "use_sim_time": use_sim_time}.items(),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "bridge.launch.py"])),
        launch_arguments={"world_name": world_name, "use_sim_time": use_sim_time}.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "rviz.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "slam.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(enable_slam),
    )

    # One-shot cleanup BEFORE starting anything. This prevents:
    # - orphaned `ign gazebo server` from previous runs
    # - orphaned `ros_gz_bridge/parameter_bridge` publishing /clock
    cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            # Use []-patterns so pkill does NOT match its own command line.
            "pkill -f '[r]os_gz_bridge/parameter_bridge' || true; "
            "pkill -f '[i]gn gazebo' || true; "
            "pkill -f 'g[z] sim' || true; "
            "pkill -f '[r]viz2/rviz2' || true; "
            "pkill -f '[r]obot_state_publisher/robot_state_publisher' || true; "
            "pkill -f '[s]lam_toolbox' || true; "
            "pkill -f '[s]tatic_transform_publisher' || true; "
            "pkill -f '[r]os2 daemon' || true; "
            # Clean up FastDDS SHM artifacts which can make new ROS2 processes hang on WSL2.
            "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true",
        ],
        output="screen",
    )

    start_everything_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[sim, bridge, rviz, slam],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="minidog_world"),
            DeclareLaunchArgument("enable_slam", default_value="true"),
            # WSL2 can have flaky multicast DDS discovery. Localhost-only makes graph discovery reliable.
            DeclareLaunchArgument("ros_localhost_only", default_value="0"),
            # If empty, use default RMW. Set to rmw_fastrtps_cpp if discovery still hangs.
            DeclareLaunchArgument("rmw_implementation", default_value=""),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profiles_file),
            cleanup,
            start_everything_after_cleanup,
        ]
    )


