from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # Gazebo -> ROS
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            [
                "/world/",
                world_name,
                "/model/minimal_4wheel_ackermann/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            "/model/minimal_4wheel_ackermann/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/minimal_4wheel_ackermann/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            # ROS -> Gazebo
            "/model/minimal_4wheel_ackermann/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        remappings=[
            ([ "/world/", world_name, "/clock" ], "/clock"),
            ([ "/world/", world_name, "/model/minimal_4wheel_ackermann/joint_state" ], "/joint_states"),
            ("/model/minimal_4wheel_ackermann/odometry", "/odom"),
            ("/model/minimal_4wheel_ackermann/tf", "/tf"),
            ("/model/minimal_4wheel_ackermann/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="minimal_4wheel_world"),
            bridge,
        ]
    )


