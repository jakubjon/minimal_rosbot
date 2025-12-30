from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    bridge_tf = LaunchConfiguration("bridge_tf")

    bridge_with_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # Gazebo -> ROS
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            [
                "/world/",
                world_name,
                "/model/minidog/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            # Keep wheel odometry available for debugging, but DON'T publish it on /odom
            # (so we can optionally run scan-matching odom on /odom).
            "/model/minidog/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # Gazebo TF provides minidog/odom -> minidog/base_footprint if you're using wheel odom.
            "/model/minidog/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/minidog/ouster/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            # ROS -> Gazebo
            "/model/minidog/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        remappings=[
            ([ "/world/", world_name, "/clock" ], "/clock"),
            ([ "/world/", world_name, "/model/minidog/joint_state" ], "/joint_states"),
            ("/model/minidog/odometry", "/wheel_odom"),
            ("/model/minidog/tf", "/tf"),
            ("/minidog/ouster/points", "/scan"),
            ("/model/minidog/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(bridge_tf),
    )

    bridge_no_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # Gazebo -> ROS
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            [
                "/world/",
                world_name,
                "/model/minidog/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            "/model/minidog/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/minidog/ouster/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            # ROS -> Gazebo
            "/model/minidog/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        remappings=[
            ([ "/world/", world_name, "/clock" ], "/clock"),
            ([ "/world/", world_name, "/model/minidog/joint_state" ], "/joint_states"),
            ("/model/minidog/odometry", "/wheel_odom"),
            ("/minidog/ouster/points", "/scan"),
            ("/model/minidog/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(bridge_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="minidog_world"),
            DeclareLaunchArgument(
                # bridge_tf=true means Gazebo provides minidog/odom->minidog/base_footprint.
                # If you run scan-matching odom (rf2o), set bridge_tf=false to avoid conflicting TF.
                "bridge_tf",
                default_value="true",
            ),
            bridge_with_tf,
            bridge_no_tf,
        ]
    )


