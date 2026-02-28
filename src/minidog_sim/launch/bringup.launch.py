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
from launch_ros.actions import Node


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
    data_source = LaunchConfiguration("data_source")
    robot_type = LaunchConfiguration("robot_type")
    bag_path = LaunchConfiguration("bag_path")
    bag_namespace = LaunchConfiguration("bag_namespace")

    # Conditions
    _is_sim = PythonExpression(["'", data_source, "' == 'sim'"])
    _is_bag = PythonExpression(["'", data_source, "' == 'bag'"])
    _is_sim_odom_wheel_or_scan = PythonExpression([
        "'", data_source, "' == 'sim' and '", odom_source, "' in ['wheel', 'scan']"
    ])
    _is_sim_odom_scan_matcher = PythonExpression([
        "'", data_source, "' == 'sim' and '", odom_source, "' == 'scan_matcher'"
    ])
    _is_sim_odom_rf2o = PythonExpression([
        "'", data_source, "' == 'sim' and '", odom_source, "' == 'rf2o'"
    ])

    # ================================================================
    # SIM-ONLY NODES: Gazebo + bridge + odom pipeline
    # ================================================================
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "sim.launch.py"])),
        launch_arguments={
            "world_name": world_name,
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "headless": headless,
            "robot_type": robot_type,
        }.items(),
        condition=IfCondition(_is_sim),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "bridge.launch.py"])),
        launch_arguments={
            "world_name": world_name,
            "use_sim_time": use_sim_time,
            "bridge_tf": "false",
            "quiet_terminal": quiet_terminal,
            "log_level": log_level,
            "robot_type": robot_type,
        }.items(),
        condition=IfCondition(_is_sim),
    )

    scan_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "scan_odom.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "mode": odom_source,
        }.items(),
        condition=IfCondition(_is_sim_odom_wheel_or_scan),
    )
    scan_matcher_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "laser_scan_matcher.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "scan_topic": "/scan",
        }.items(),
        condition=IfCondition(_is_sim_odom_scan_matcher),
    )

    scan_filter = Node(
        package="minidog_sim",
        executable="scan_filter.py",
        name="scan_safety_filter",
        output="screen",
        remappings=[
            ("scan_raw", "/scan"),
            ("scan_safe", "/scan_safe"),
        ],
        condition=IfCondition(_is_sim_odom_rf2o),
    )

    rf2o_odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan_safe",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(_is_sim_odom_rf2o),
    )

    odom_stabilizer = Node(
        package="minidog_sim",
        executable="odom_stabilizer.py",
        name="odom_stabilizer",
        output="screen",
        parameters=[
            {
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": True,
                "freq": 20.0,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("odom_rf2o", "/odom_rf2o"),
            ("odom", "/odom"),
        ],
        condition=IfCondition(_is_sim_odom_rf2o),
    )

    # ================================================================
    # BAG-ONLY NODES: rosbag play + data_relay
    # ================================================================
    bag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag_path,
            "--clock",
            "--rate", "1.0",
        ],
        output="screen",
        condition=IfCondition(_is_bag),
    )

    data_relay = Node(
        package="minidog_sim",
        executable="data_relay.py",
        name="data_relay",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "namespace": bag_namespace,
        }],
        condition=IfCondition(_is_bag),
    )

    # ================================================================
    # SHARED NODES: RViz/RSP, mux, SLAM, Nav2, explore, web
    # ================================================================
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", "rviz.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "enable_rviz_gui": enable_rviz,
            "data_source": data_source,
            "robot_type": robot_type,
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
            "robot_type": robot_type,
        }.items(),
        condition=IfCondition(enable_nav2),
    )

    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("minidog_explore"), "launch", "explore.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "quiet_terminal": quiet_terminal,
            "heading_weight": PythonExpression([
                "'0.5' if '", robot_type, "' == 'diffbot' else '2.0'"
            ]),
        }.items(),
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
            "  pkill -9 -f '[d]ata_relay' || true; "
            "  [ $i -eq 1 ] && sleep 1; "
            "done; "
            "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true",
        ],
        output="screen",
    )

    # ----------------------------------------------------------------
    # Staggered launch after cleanup
    # SIM:  Gazebo(0s) -> odom(0.5s) -> rviz+mux(1.5s) -> SLAM(5s) -> Nav2(10s) -> explore(20s)
    # BAG:  bag+relay(0s) -> rviz(1s) -> SLAM(3s) -> Nav2(8s) -> explore(15s)
    # ----------------------------------------------------------------
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                # === SIM MODE PHASES ===
                # Phase 1: Gazebo + bridge + wheel odom + scan matcher
                sim,
                bridge,
                scan_odom,
                scan_matcher_odom,
                # Phase 2a: scan filter and rf2o after 0.5s
                TimerAction(period=0.5, actions=[scan_filter, rf2o_odom]),
                # Phase 2b: odom stabilizer after 1.0s
                TimerAction(period=1.0, actions=[odom_stabilizer]),

                # === BAG MODE PHASES ===
                # Phase 1: rosbag play + data_relay
                bag_play,
                data_relay,

                # === SHARED PHASES ===
                # RViz + mux (sim: 1.5s, bag: 1.0s — both work with 1.5s)
                TimerAction(period=1.5, actions=[mux, rviz]),
                # SLAM (sim: 5s for Gazebo startup, bag: 3s is enough)
                TimerAction(period=5.0, actions=[slam]),
                # Nav2 (sim: 10s, bag: 8s — 10s safe for both)
                TimerAction(period=10.0, actions=[nav2]),
                # Explore + web (sim: 20s, bag: 15s — 20s safe for both)
                TimerAction(period=20.0, actions=[explore, web, rqt]),
            ],
        )
    )

    return LaunchDescription(
        [
            # Core args
            DeclareLaunchArgument("data_source", default_value="sim",
                                 description="Data source: 'sim' (Gazebo) or 'bag' (rosbag replay)"),
            DeclareLaunchArgument("robot_type", default_value="diffbot",
                                 description="Robot type: 'diffbot' (differential drive) or 'ackermann'"),
            DeclareLaunchArgument("bag_path",
                                 default_value="/mnt/c/Users/Jakub/Desktop/20260115_3",
                                 description="Path to rosbag directory"),
            DeclareLaunchArgument("bag_namespace", default_value="go2_unit_27778",
                                 description="GO2 robot namespace in rosbag"),
            # Existing args
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
            DeclareLaunchArgument("odom_source", default_value="rf2o"),
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
