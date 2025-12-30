from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("minidog_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    quiet_terminal = LaunchConfiguration("quiet_terminal")
    log_level = LaunchConfiguration("log_level")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_mux = LaunchConfiguration("enable_mux")
    enable_explore = LaunchConfiguration("enable_explore")
    enable_rqt = LaunchConfiguration("enable_rqt")
    enable_web = LaunchConfiguration("enable_web")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    odom_source = LaunchConfiguration("odom_source")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    rmw_implementation = LaunchConfiguration("rmw_implementation")
    fastdds_profiles_file = PathJoinSubstitution([pkg_share, "config", "fastdds_no_shm.xml"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "sim.launch.py"])),
        launch_arguments={
            "world_name": world_name,
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
        }.items(),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "bridge.launch.py"])),
        # If we use scan-matching odom, DO NOT bridge Gazebo TF (it would conflict on minidog/odom).
        launch_arguments={
            "world_name": world_name,
            "use_sim_time": use_sim_time,
            "bridge_tf": PythonExpression(["'", odom_source, "' == 'wheel'"]),
            "quiet_terminal": quiet_terminal,
            "log_level": log_level,
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "rviz.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time, "quiet_terminal": quiet_terminal}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "slam.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time, "quiet_terminal": quiet_terminal}.items(),
        condition=IfCondition(enable_slam),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "nav2.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "log_level": log_level,
        }.items(),
        condition=IfCondition(enable_nav2),
    )
    mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("minidog_cmd_mux"), "launch", "mux.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "manual_topic": "/cmd_vel_manual",
            "auto_topic": "/cmd_vel_nav",
            "output_topic": "/cmd_vel",
            "enable_topic": "/autonomy_enabled",
        }.items(),
        condition=IfCondition(enable_mux),
    )
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("minidog_explore"), "launch", "explore.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "quiet_terminal": quiet_terminal}.items(),
        condition=IfCondition(enable_explore),
    )
    rqt = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            # Fail-safe: don't crash bringup if rqt isn't installed.
            # --force-discover helps on some systems where the daemon/graph can be flaky.
            "command -v rqt >/dev/null 2>&1 && rqt --force-discover || true",
        ],
        output="screen",
        condition=IfCondition(enable_rqt),
    )

    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "web.launch.py"])
        ),
        launch_arguments={"host": web_host, "port": web_port}.items(),
        condition=IfCondition(enable_web),
    )
    scan_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "scan_odom.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odom_frame": "minidog/odom",
            "base_frame": "minidog/base_footprint",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
        }.items(),
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'scan'"])),
    )
    scan_matcher_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "laser_scan_matcher.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "scan_topic": "/scan",
        }.items(),
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'scan_matcher'"])),
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
            "pkill -f '[n]av2_' || true; "
            "pkill -f '[s]tatic_transform_publisher' || true; "
            # rf2o process name varies (rf2o_laser_odometry_node / rf2o_laser_odometry)
            "pkill -f '[r]f2o_laser_odometry_node' || true; "
            "pkill -f '[r]f2o_laser_odometry' || true; "
            "pkill -f '[l]aser_scan_matcher' || true; "
            "pkill -f '[c]md_vel_mux' || true; "
            "pkill -f '[f]rontier_explore' || true; "
            "pkill -f '[r]qt' || true; "
            "pkill -f '[r]os2 daemon' || true; "
            # Clean up FastDDS SHM artifacts which can make new ROS2 processes hang on WSL2.
            "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true",
        ],
        output="screen",
    )

    start_everything_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[sim, bridge, scan_odom, scan_matcher_odom, mux, rviz, slam, nav2, explore, rqt, web],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="minidog_world"),
            DeclareLaunchArgument("quiet_terminal", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument("enable_slam", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="false"),
            DeclareLaunchArgument("enable_mux", default_value="true"),
            DeclareLaunchArgument("enable_explore", default_value="false"),
            DeclareLaunchArgument("enable_rqt", default_value="false"),
            DeclareLaunchArgument("enable_web", default_value="false"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8501"),
            DeclareLaunchArgument(
                # wheel: odom->base comes from Gazebo TF (wheel-integrated).
                # scan:  odom->base comes from scan matching (rf2o), wheel odom is still available as /wheel_odom.
                # scan_matcher: odom->base comes from scan matching (laser_scan_matcher). Requires a ROS 2 port installed.
                "odom_source",
                default_value="wheel",
            ),
            # WSL2 can have flaky multicast DDS discovery. Localhost-only makes graph discovery reliable.
            DeclareLaunchArgument("ros_localhost_only", default_value="0"),
            # If empty, use default RMW. Set to rmw_fastrtps_cpp if discovery still hangs.
            DeclareLaunchArgument("rmw_implementation", default_value=""),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profiles_file),
            # qre_go2 pattern: ensure stdout logs flush line-buffered (better behavior under launch/logging).
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            cleanup,
            start_everything_after_cleanup,
        ]
    )


