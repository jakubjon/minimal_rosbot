from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    quiet_terminal = LaunchConfiguration("quiet_terminal")
    enable_rviz_gui = LaunchConfiguration("enable_rviz_gui")
    data_source = LaunchConfiguration("data_source")
    robot_type = LaunchConfiguration("robot_type")

    pkg_share = FindPackageShare("minidog_sim")

    # Select URDF based on robot_type
    diffbot_xacro = PathJoinSubstitution([pkg_share, "urdf", "diffbot.urdf.xacro"])
    minidog_xacro = PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])

    rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", "robot.rviz"])
    rviz_bag_config_path = PathJoinSubstitution([pkg_share, "rviz", "robot_bag.rviz"])

    # Conditions
    _is_diffbot_sim = PythonExpression(["'", data_source, "' == 'sim' and '", robot_type, "' == 'diffbot'"])
    _is_acker_sim = PythonExpression(["'", data_source, "' == 'sim' and '", robot_type, "' != 'diffbot'"])

    diffbot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", diffbot_xacro]),
        value_type=str,
    )
    minidog_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", minidog_xacro]),
        value_type=str,
    )

    # RSP only runs in sim mode (bag mode gets TF from data_relay)
    rsp_diffbot = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": diffbot_description,
        }],
        condition=IfCondition(_is_diffbot_sim),
    )

    rsp_ackermann = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": minidog_description,
        }],
        condition=IfCondition(_is_acker_sim),
    )

    # Gazebo publishes scans with frame_id "{model_name}/base_footprint/ouster".
    # Provide static TF from RSP's base_footprint to Gazebo's scan frame.
    ouster_tf_diffbot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ouster_static_tf",
        output="log",
        arguments=[
            "--x", "0.15", "--y", "0.0", "--z", "0.20",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "diffbot/base_footprint/ouster",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_is_diffbot_sim),
    )

    ouster_tf_ackermann = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ouster_static_tf",
        output="log",
        arguments=[
            "--x", "0.1", "--y", "0.0", "--z", "0.16",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "minidog/base_footprint/ouster",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_is_acker_sim),
    )

    # RViz GUI — select config based on data_source
    rviz_sim = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", enable_rviz_gui, "' == 'true' and '", data_source, "' == 'sim'"
        ])),
    )

    rviz_bag = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_bag_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", enable_rviz_gui, "' == 'true' and '", data_source, "' == 'bag'"
        ])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
            DeclareLaunchArgument("enable_rviz_gui", default_value="true"),
            DeclareLaunchArgument("data_source", default_value="sim"),
            DeclareLaunchArgument("robot_type", default_value="diffbot"),
            # RSP + static TF (sim only)
            rsp_diffbot,
            rsp_ackermann,
            ouster_tf_diffbot,
            ouster_tf_ackermann,
            # RViz (conditional on data_source)
            rviz_sim,
            rviz_bag,
        ]
    )
