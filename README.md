# minidog_sim (ROS 2 Humble + Gazebo Fortress)

Colcon workspace containing a minimal 4-wheel **Ackermann** robot simulated in **Gazebo Fortress / Ignition Gazebo 6**, driven by **ROS 2 `/cmd_vel`**, and visualized in **RViz**.

## Requirements
- ROS 2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- `ros_gz_sim`, `ros_gz_bridge`

## Build

```bash
cd /home/jjon/ROBOPES/minimal
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Run (single bringup)

This launch file first cleans up common orphan processes (Gazebo + bridges) and then starts:
- Gazebo GUI + spawns the robot
- ros_gz bridges (`/cmd_vel`, `/clock`, `/joint_states`, `/odom`, `/tf`)
- `robot_state_publisher`
- RViz (with correct TF prefix)

```bash
cd /home/jjon/ROBOPES/minimal
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch minidog_sim bringup.launch.py
```

## Drive

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 10 -t 2
```

## WSL note (ros2 CLI hangs)

On some WSL setups the ROS 2 CLI may hang due to a stuck `ros2 daemon`. Use `--no-daemon`:

```bash
ros2 topic list --no-daemon
```


