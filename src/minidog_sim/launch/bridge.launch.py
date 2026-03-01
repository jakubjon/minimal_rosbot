from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    bridge_tf = LaunchConfiguration("bridge_tf")
    quiet_terminal = LaunchConfiguration("quiet_terminal")
    log_level = LaunchConfiguration("log_level")
    robot_type = LaunchConfiguration("robot_type")

    # Gazebo topic paths depend on the model name (diffbot vs minidog)
    # We create bridge nodes for each variant, conditioned on robot_type.
    _is_diffbot = PythonExpression(["'", robot_type, "' == 'diffbot'"])
    _is_ackermann = PythonExpression(["'", robot_type, "' != 'diffbot'"])

    # Helper to avoid repetition — bridge_tf is always false for the no-tf variant
    _diffbot_with_tf = PythonExpression([
        "'", robot_type, "' == 'diffbot' and '", bridge_tf, "' == 'true'"
    ])
    _diffbot_no_tf = PythonExpression([
        "'", robot_type, "' == 'diffbot' and '", bridge_tf, "' != 'true'"
    ])
    _acker_with_tf = PythonExpression([
        "'", robot_type, "' != 'diffbot' and '", bridge_tf, "' == 'true'"
    ])
    _acker_no_tf = PythonExpression([
        "'", robot_type, "' != 'diffbot' and '", bridge_tf, "' != 'true'"
    ])

    # ---- DIFFBOT bridge (with TF) ----
    bridge_diffbot_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            ["/world/", world_name, "/model/diffbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"],
            "/model/diffbot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/diffbot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/diffbot/ouster/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/diffbot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/model/diffbot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "--ros-args", "--log-level", log_level,
        ],
        remappings=[
            (["/world/", world_name, "/clock"], "/clock"),
            (["/world/", world_name, "/model/diffbot/joint_state"], "/joint_states"),
            ("/model/diffbot/odometry", "/wheel_odom"),
            ("/model/diffbot/tf", "/tf"),
            ("/diffbot/ouster/points", "/scan"),
            ("/diffbot/imu", "/imu/bridge_raw"),
            ("/model/diffbot/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_diffbot_with_tf),
    )

    # ---- DIFFBOT bridge (no TF) ----
    bridge_diffbot_no_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            ["/world/", world_name, "/model/diffbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"],
            "/model/diffbot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/diffbot/ouster/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/diffbot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/model/diffbot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "--ros-args", "--log-level", log_level,
        ],
        remappings=[
            (["/world/", world_name, "/clock"], "/clock"),
            (["/world/", world_name, "/model/diffbot/joint_state"], "/joint_states"),
            ("/model/diffbot/odometry", "/wheel_odom"),
            ("/diffbot/ouster/points", "/scan"),
            ("/diffbot/imu", "/imu/bridge_raw"),
            ("/model/diffbot/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_diffbot_no_tf),
    )

    # ---- ACKERMANN (minidog) bridge (with TF) ----
    bridge_acker_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            ["/world/", world_name, "/model/minidog/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"],
            "/model/minidog/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/minidog/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/minidog/ouster/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/model/minidog/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "--ros-args", "--log-level", log_level,
        ],
        remappings=[
            (["/world/", world_name, "/clock"], "/clock"),
            (["/world/", world_name, "/model/minidog/joint_state"], "/joint_states"),
            ("/model/minidog/odometry", "/wheel_odom"),
            ("/model/minidog/tf", "/tf"),
            ("/minidog/ouster/points", "/scan"),
            ("/model/minidog/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_acker_with_tf),
    )

    # ---- ACKERMANN (minidog) bridge (no TF) ----
    bridge_acker_no_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            ["/world/", world_name, "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            ["/world/", world_name, "/model/minidog/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"],
            "/model/minidog/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/minidog/ouster/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/model/minidog/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "--ros-args", "--log-level", log_level,
        ],
        remappings=[
            (["/world/", world_name, "/clock"], "/clock"),
            (["/world/", world_name, "/model/minidog/joint_state"], "/joint_states"),
            ("/model/minidog/odometry", "/wheel_odom"),
            ("/minidog/ouster/points", "/scan"),
            ("/model/minidog/cmd_vel", "/cmd_vel"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_acker_no_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="minidog_world"),
            DeclareLaunchArgument("quiet_terminal", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument("bridge_tf", default_value="true"),
            DeclareLaunchArgument("robot_type", default_value="diffbot"),
            bridge_diffbot_tf,
            bridge_diffbot_no_tf,
            bridge_acker_tf,
            bridge_acker_no_tf,
        ]
    )
