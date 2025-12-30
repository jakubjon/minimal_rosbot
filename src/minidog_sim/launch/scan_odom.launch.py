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

    # NOTE:
    # This requires installing: ros-humble-rf2o-laser-odometry
    # (we can't sudo-install from this environment).
    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"publish_tf": True},
            {"odom_frame_id": odom_frame},
            {"base_frame_id": base_frame},
            {"laser_scan_topic": scan_topic},
            # In this sim we don't have ground-truth / base_pose topic; start at identity.
            {"init_pose_from_topic": ""},
            # Ensure rf2o publishes on the expected odom topic.
            {"odom_topic": odom_topic},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("odom_frame", default_value="minidog/odom"),
            DeclareLaunchArgument("base_frame", default_value="minidog/base_footprint"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            rf2o,
        ]
    )


