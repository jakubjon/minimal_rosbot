from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    quiet_terminal = LaunchConfiguration("quiet_terminal")
    enable_rviz_gui = LaunchConfiguration("enable_rviz_gui")

    pkg_share = FindPackageShare("minidog_sim")
    robot_xacro_path = PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", "robot.rviz"])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_xacro_path]),
        value_type=str,
    )

    # robot_state_publisher ALWAYS runs (needed for TF tree).
    rsp_screen = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
                # Gazebo publishes odom / base_footprint with model-name prefix.
                # Match that so RViz has a connected TF tree.
                "frame_prefix": "minidog/",
            }
        ],
        condition=UnlessCondition(quiet_terminal),
    )
    rsp_log = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
                "frame_prefix": "minidog/",
            }
        ],
        condition=IfCondition(quiet_terminal),
    )

    # Gazebo (via ros_gz_bridge) publishes /scan with frame_id: "minidog/base_footprint/ouster".
    # Provide that TF frame so slam_toolbox + RViz can transform scans.
    # ALWAYS runs (needed for SLAM).
    ouster_frame_tf_screen = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="minidog_ouster_static_tf",
        output="screen",
        arguments=[
            "--x", "0.1",
            "--y", "0.0",
            "--z", "0.16",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "minidog/base_footprint",
            "--child-frame-id", "minidog/base_footprint/ouster",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(quiet_terminal),
    )
    ouster_frame_tf_log = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="minidog_ouster_static_tf",
        output="log",
        arguments=[
            "--x", "0.1",
            "--y", "0.0",
            "--z", "0.16",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "minidog/base_footprint",
            "--child-frame-id", "minidog/base_footprint/ouster",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(quiet_terminal),
    )

    # RViz GUI is optional (heavy on CPU/GPU in WSL2).
    rviz_gui = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_rviz_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
            DeclareLaunchArgument("enable_rviz_gui", default_value="true"),
            # TF publishers always run (needed by SLAM + Nav2)
            rsp_screen,
            rsp_log,
            ouster_frame_tf_screen,
            ouster_frame_tf_log,
            # RViz GUI is conditional
            rviz_gui,
        ]
    )
