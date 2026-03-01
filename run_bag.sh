#!/usr/bin/env bash
set -e

# Replay a real GO2 rosbag through the SLAM/Nav2/explore pipeline.
# Usage:
#   bash run_bag.sh                          # defaults
#   bash run_bag.sh /path/to/bag             # custom bag path
#   bash run_bag.sh /path/to/bag my_ns       # custom bag path + namespace

BAG_PATH="${1:-$HOME/Desktop/20260115_3}"
BAG_NS="${2:-go2_unit_27778}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch minidog_sim bringup.launch.py \
    data_source:=bag \
    bag_path:="$BAG_PATH" \
    bag_namespace:="$BAG_NS" \
    enable_slam:=true \
    enable_nav2:=false \
    enable_explore:=false \
    enable_mux:=false \
    enable_web:=false \
    enable_rviz:=true \
    enable_ekf:=true \
    quiet_terminal:=false
