from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")

    default_params = PathJoinSubstitution(
        [FindPackageShare("minidog_sim"), "config", "laser_scan_matcher.yaml"]
    )

    # NOTE:
    # laser_scan_matcher is primarily documented for ROS1 here:
    # https://wiki.ros.org/laser_scan_matcher
    # If you have a ROS 2 port installed in your environment, this launch will work.
    lsm = Node(
        package="laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_scan_matcher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, params_file],
        remappings=[
            ("/scan", scan_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            lsm,
        ]
    )


