from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    heading_weight = LaunchConfiguration("heading_weight")
    quiet_terminal = LaunchConfiguration("quiet_terminal")

    explorer = Node(
        package="minidog_explore",
        executable="frontier_explore",
        name="frontier_explore",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "heading_weight": heading_weight,
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("heading_weight", default_value="2.0"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
            explorer,
        ]
    )


