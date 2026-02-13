from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
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
    headless = LaunchConfiguration("headless")
    enable_rviz = LaunchConfiguration("enable_rviz")

    # ----------------------------------------------------------------
    # Phase 1: Gazebo + bridge (start first, need time to initialize)
    # ----------------------------------------------------------------
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "sim.launch.py"])),
        launch_arguments={
            "world_name": world_name,
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "headless": headless,
        }.items(),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "bridge.launch.py"])),
        launch_arguments={
            "world_name": world_name,
            "use_sim_time": use_sim_time,
            "bridge_tf": "false",
            "quiet_terminal": quiet_terminal,
            "log_level": log_level,
        }.items(),
    )

    # ----------------------------------------------------------------
    # Phase 2: TF publishers + odom (need bridge for clock)
    # ----------------------------------------------------------------
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "rviz.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "enable_rviz_gui": enable_rviz,
        }.items(),
    )
    scan_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "scan_odom.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odom_frame": "minidog/odom",
            "base_frame": "minidog/base_footprint",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "mode": odom_source,
        }.items(),
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

    # ----------------------------------------------------------------
    # Phase 3: SLAM (needs TF + scan, delay 5s)
    # ----------------------------------------------------------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "slam.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time, "quiet_terminal": quiet_terminal}.items(),
        condition=IfCondition(enable_slam),
    )

    # ----------------------------------------------------------------
    # Phase 4: Nav2 (needs SLAM map + TF, delay 10s)
    # ----------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "nav2.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "log_level": log_level,
        }.items(),
        condition=IfCondition(enable_nav2),
    )

    # ----------------------------------------------------------------
    # Phase 5: Explore + Web (needs Nav2 active, delay 15s)
    # ----------------------------------------------------------------
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("minidog_explore"), "launch", "explore.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "quiet_terminal": quiet_terminal}.items(),
        condition=IfCondition(enable_explore),
    )
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "web.launch.py"])
        ),
        launch_arguments={"host": web_host, "port": web_port}.items(),
        condition=IfCondition(enable_web),
    )
    rqt = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "command -v rqt >/dev/null 2>&1 && rqt --force-discover || true",
        ],
        output="screen",
        condition=IfCondition(enable_rqt),
    )

    # ----------------------------------------------------------------
    # Cleanup: kill zombies from previous runs
    # ----------------------------------------------------------------
    cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "for i in 1 2; do "
            "  pkill -9 -f '[p]arameter_bridge' || true; "
            "  pkill -9 -f '[i]gn gazebo' || true; "
            "  pkill -9 -f 'g[z] sim' || true; "
            "  pkill -9 -f '[r]viz2' || true; "
            "  pkill -9 -f '[r]obot_state_publisher' || true; "
            "  pkill -9 -f '[a]sync_slam_toolbox_node' || true; "
            "  pkill -9 -f '[s]lam_toolbox' || true; "
            "  pkill -9 -f '[c]ontroller_server' || true; "
            "  pkill -9 -f '[p]lanner_server' || true; "
            "  pkill -9 -f '[b]t_navigator' || true; "
            "  pkill -9 -f '[b]ehavior_server' || true; "
            "  pkill -9 -f '[w]aypoint_follower' || true; "
            "  pkill -9 -f '[l]ifecycle_manager' || true; "
            "  pkill -9 -f '[s]tatic_transform_publisher' || true; "
            "  pkill -9 -f '[r]f2o_laser_odometry' || true; "
            "  pkill -9 -f '[l]aser_scan_matcher' || true; "
            "  pkill -9 -f '[s]can_odom' || true; "
            "  pkill -9 -f '[c]md_vel_mux' || true; "
            "  pkill -9 -f '[f]rontier_explore' || true; "
            "  pkill -9 -f '[s]treamlit' || true; "
            "  pkill -9 -f '[r]qt' || true; "
            "  [ $i -eq 1 ] && sleep 1; "
            "done; "
            "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true",
        ],
        output="screen",
    )

    # ----------------------------------------------------------------
    # Staggered launch after cleanup
    # Phase 1 (0s):  Gazebo + bridge
    # Phase 2 (0s):  TF publishers + odom + mux
    # Phase 3 (5s):  SLAM
    # Phase 4 (10s): Nav2
    # Phase 5 (20s): Explore + web
    # ----------------------------------------------------------------
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                # Phase 1+2: immediate
                sim,
                bridge,
                scan_odom,
                scan_matcher_odom,
                mux,
                rviz,
                # Phase 3: SLAM after 5s
                TimerAction(period=5.0, actions=[slam]),
                # Phase 4: Nav2 after 10s (SLAM needs time to produce first map)
                TimerAction(period=10.0, actions=[nav2]),
                # Phase 5: Explore + web after 20s (Nav2 needs lifecycle bringup)
                TimerAction(period=20.0, actions=[explore, web, rqt]),
            ],
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
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("enable_rqt", default_value="false"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("enable_web", default_value="false"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8501"),
            DeclareLaunchArgument("odom_source", default_value="scan"),
            DeclareLaunchArgument("ros_localhost_only", default_value="0"),
            DeclareLaunchArgument("rmw_implementation", default_value=""),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profiles_file),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            cleanup,
            start_after_cleanup,
        ]
    )
