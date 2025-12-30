from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")

    app_path = PathJoinSubstitution([FindPackageShare("minidog_sim"), "webapp", "app.py"])

    # Requires: pip install -r $(ros2 pkg prefix minidog_sim)/share/minidog_sim/webapp/requirements.txt
    streamlit = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "python3 -m streamlit run "
            + str(app_path)
            + " --server.address "
            + str(host)
            + " --server.port "
            + str(port)
            + " --server.headless true",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8501"),
            streamlit,
        ]
    )


