from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    mode = LaunchConfiguration("mode")

    # Odometry TF publisher.
    # - mode=wheel: subscribes to /wheel_odom, publishes proper odom->base TF + /odom
    # - mode=scan:  publishes identity odom->base TF (SLAM-delegated)
    scan_odom = Node(
        package="minidog_scan_odom",
        executable="scan_odom_node",
        name="scan_odom",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"publish_tf": True},
            {"mode": mode},
            {"odom_frame_id": odom_frame},
            {"base_frame_id": base_frame},
            {"laser_scan_topic": scan_topic},
            {"odom_topic": odom_topic},
            {"wheel_odom_topic": "/wheel_odom"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("odom_frame", default_value="minidog/odom"),
            DeclareLaunchArgument("base_frame", default_value="minidog/base_footprint"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("mode", default_value="wheel"),
            scan_odom,
        ]
    )
