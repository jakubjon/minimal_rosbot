#!/usr/bin/env bash
set -e

# ------------------------------------------------------------------
# Kill zombies from previous runs (aggressive, run twice)
# ------------------------------------------------------------------
kill_zombies() {
    # Kill by full command-line match (covers Python scripts and C++ nodes)
    pkill -9 -f 'parameter_bridge'               2>/dev/null || true
    pkill -9 -f 'ign gazebo'                      2>/dev/null || true
    pkill -9 -f 'gz sim'                          2>/dev/null || true
    pkill -9 -f 'rviz2'                           2>/dev/null || true
    pkill -9 -f 'robot_state_publisher'            2>/dev/null || true
    pkill -9 -f 'async_slam_toolbox_node'          2>/dev/null || true
    pkill -9 -f 'slam_toolbox'                     2>/dev/null || true
    pkill -9 -f 'controller_server'                2>/dev/null || true
    pkill -9 -f 'planner_server'                   2>/dev/null || true
    pkill -9 -f 'bt_navigator'                     2>/dev/null || true
    pkill -9 -f 'behavior_server'                  2>/dev/null || true
    pkill -9 -f 'waypoint_follower'                2>/dev/null || true
    pkill -9 -f 'lifecycle_manager'                2>/dev/null || true
    pkill -9 -f 'static_transform_publisher'       2>/dev/null || true
    pkill -9 -f 'rf2o_laser_odometry'              2>/dev/null || true
    pkill -9 -f 'laser_scan_matcher'               2>/dev/null || true
    pkill -9 -f 'cmd_vel_mux'                      2>/dev/null || true
    pkill -9 -f 'frontier_explore'                 2>/dev/null || true
    pkill -9 -f 'streamlit'                        2>/dev/null || true
    pkill -9 -f 'minidog_sim'                      2>/dev/null || true
    pkill -9 -f 'minidog_explore'                  2>/dev/null || true
    pkill -9 -f 'minidog_cmd_mux'                  2>/dev/null || true
    pkill -9 -f 'ros2 launch'                     2>/dev/null || true
    pkill -9 -f 'rqt'                             2>/dev/null || true
}

# Run cleanup twice — first pass kills main processes, second pass
# catches any children that got reparented to PID 1 after their
# parent was killed.
kill_zombies
sleep 1
kill_zombies

# Clean up DDS shared-memory artifacts (can make ROS 2 processes hang on WSL2)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 1

# ------------------------------------------------------------------
# Source ROS + workspace, build, launch
# ------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch minidog_sim bringup.launch.py \
    robot_type:=diffbot \
    odom_source:=rf2o \
    enable_slam:=true \
    enable_nav2:=true \
    enable_explore:=true \
    enable_web:=true \
    enable_rviz:=true \
    headless:=false \
    enable_ekf:=true \
    quiet_terminal:=false
