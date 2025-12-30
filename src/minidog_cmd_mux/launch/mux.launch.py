from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    manual_topic = LaunchConfiguration("manual_topic")
    auto_topic = LaunchConfiguration("auto_topic")
    output_topic = LaunchConfiguration("output_topic")
    enable_topic = LaunchConfiguration("enable_topic")

    mux = Node(
        package="minidog_cmd_mux",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "manual_topic": manual_topic,
                "auto_topic": auto_topic,
                "output_topic": output_topic,
                "enable_topic": enable_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("manual_topic", default_value="/cmd_vel_manual"),
            DeclareLaunchArgument("auto_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("enable_topic", default_value="/autonomy_enabled"),
            mux,
        ]
    )


