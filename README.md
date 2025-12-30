# minidog_sim (ROS 2 Humble + Gazebo Fortress)

Colcon workspace containing a minimal 4-wheel **Ackermann** robot simulated in **Gazebo Fortress / Ignition Gazebo 6**, driven by **ROS 2 `/cmd_vel`**, and visualized in **RViz**.

This repo is a **sandbox** designed to follow the modular integration logic from `qre_go2/` (reference only) while using a different odometry source:
- **qre_go2 reference**: modular launches, strict TF ownership, stable topic remappings.
- **minidog difference**: odometry can come from **scan matching** via **`rf2o_laser_odometry`** (vendored under `src/third_party/`).

Ultimate target: **autonomous exploration of unknown space while building a map** (online SLAM + Nav2 + frontier exploration), with **manual mode as the default**.

## Requirements
- ROS 2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- `ros_gz_sim`, `ros_gz_bridge`
- Nav2 (`nav2_bringup` and related packages)

## Build

```bash
cd /home/jjon/ROBOPES/minimal
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/minidog_sim src/minidog_cmd_mux src/minidog_explore src/third_party/rf2o_laser_odometry
```

If you previously renamed packages or removed `install/` and now see warnings about non-existent paths in `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH`, do:
```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
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

Important: source the **workspace overlay** (`install/setup.bash`), not a single-package local setup file.


### Manual drive (default)

This repo uses `minidog_cmd_mux` to ensure **manual is the default** and autonomous control is only active when explicitly enabled:
- **Manual input**: `/cmd_vel_manual`
- **Autonomous input (Nav2)**: `/cmd_vel_nav`
- **Output to robot / sim**: `/cmd_vel`
- **Enable switch**: `/autonomy_enabled` (`std_msgs/msg/Bool`)

When `/autonomy_enabled` is `false`, `/cmd_vel_manual` controls the robot. When `true`, `/cmd_vel_nav` controls the robot.

```bash
ros2 topic pub /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 10 -t 2
```

### Scan odometry (rf2o) + SLAM

This repo is set up to use **rf2o scan-matching odometry** as the preferred odom source.

What this means (TF ownership):
- rf2o publishes **`minidog/odom -> minidog/base_footprint`**
- slam_toolbox publishes **`map -> minidog/odom`**
- Gazebo wheel odometry is kept only as **`/wheel_odom`** for debugging.

#### Build rf2o (vendored)

rf2o is included as a git submodule under `src/third_party/rf2o_laser_odometry` and built as part of the workspace.

After cloning this repo, fetch submodules:

```bash
git submodule update --init --recursive
```

```bash
cd /home/jjon/ROBOPES/minimal
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/minidog_sim src/minidog_cmd_mux src/minidog_explore src/third_party/rf2o_laser_odometry
. install/setup.bash
ros2 pkg executables rf2o_laser_odometry
```

#### Run with rf2o

Use scan odometry by setting `odom_source:=scan`:

```bash
ros2 launch minidog_sim bringup.launch.py odom_source:=scan
```

#### rf2o parameters (what we set for simulation)

rf2o is configured in `src/minidog_sim/launch/scan_odom.launch.py` with simulation-friendly defaults:
- `laser_scan_topic: /scan`
- `odom_topic: /odom` (rf2o default is `/odom_rf2o`, but we standardize on `/odom`)
- `odom_frame_id: minidog/odom`
- `base_frame_id: minidog/base_footprint`
- `publish_tf: true`
- `init_pose_from_topic: ""` (we don’t have ground-truth init pose in this sim; rf2o should start at identity)

### Enable autonomous frontier exploration (manual activation)

Start Nav2 + frontier exploration (recommended with scan odometry):

```bash
ros2 launch minidog_sim bringup.launch.py odom_source:=scan enable_nav2:=true enable_explore:=true
```

Autonomy is **off by default**. Enable it:

```bash
ros2 topic pub /autonomy_enabled std_msgs/msg/Bool "{data: true}" -r 2
```

Disable it (back to manual):

```bash
ros2 topic pub /autonomy_enabled std_msgs/msg/Bool "{data: false}" -r 2
```

Notes:
- Nav2 velocity output is `/cmd_vel_nav`. Manual input is `/cmd_vel_manual`. The mux publishes the final `/cmd_vel` that Gazebo consumes. Avoid publishing directly to `/cmd_vel` from teleop to prevent conflicts.
- Wheel odometry from Gazebo is bridged to `/wheel_odom` for debugging. When using `odom_source:=scan`, Gazebo TF bridging is disabled to avoid TF conflicts.

## WSL note (ros2 CLI hangs)

On some WSL setups the ROS 2 CLI may hang due to a stuck `ros2 daemon`. Use `--no-daemon`:

```bash
ros2 topic list --no-daemon
```


