# minidog_sim (ROS 2 Humble + Gazebo Fortress)

Minimal 4-wheel **Ackermann** robot simulation with **fully autonomous frontier exploration**. The robot starts in a 3-room arena, explores all accessible space using SLAM + Nav2 + a custom frontier explorer, and announces completion when done.

Stack: **ROS 2 Humble**, **Gazebo Fortress (Ignition 6)**, **slam_toolbox**, **Nav2**, **Streamlit web UI**.

## Quick start

```bash
cd /home/jjon/ROBOPES/minimal
./run.sh
```

`run.sh` kills zombie processes from previous runs, builds the workspace, and launches the full stack in headless mode with autonomy auto-enabled. The robot starts exploring immediately. Watch the terminal for `EXPLORATION COMPLETE`.

Web UI available at `http://localhost:8501`.

## Build (manual)

```bash
cd /home/jjon/ROBOPES/minimal
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Launch (manual)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch minidog_sim bringup.launch.py \
    odom_source:=wheel \
    enable_slam:=true \
    enable_nav2:=true \
    enable_explore:=true \
    enable_web:=true \
    enable_rviz:=false \
    headless:=true
```

### Key launch arguments

| Argument | Default | Description |
|---|---|---|
| `odom_source` | `scan` | `wheel` (Gazebo GT), `scan` (rf2o), `scan_matcher` |
| `enable_slam` | `true` | Start `slam_toolbox` for mapping |
| `enable_nav2` | `false` | Start Nav2 navigation stack |
| `enable_explore` | `false` | Start frontier exploration node |
| `enable_web` | `false` | Start Streamlit web UI on port 8501 |
| `enable_rviz` | `true` | Start RViz visualization |
| `headless` | `false` | Run Gazebo without GUI (server-only) |
| `quiet_terminal` | `true` | Route node output to log files |
| `log_level` | `warn` | Nav2/bridge log verbosity |

### Staggered startup

Components start in phases to avoid race conditions:
- **0s**: Gazebo + bridge + TF + odometry + mux
- **5s**: SLAM
- **10s**: Nav2 (+ 5s internal lifecycle manager delay)
- **20s**: Frontier explorer + web UI

### Logging

Most output goes to `~/.ros/log/latest/`. For verbose terminal:

```bash
ros2 launch minidog_sim bringup.launch.py quiet_terminal:=false log_level:=info
```

## World

Three-room arena (`minidog_world.sdf`):
- **Room 1** (6x6m): robot spawn at origin
- **Room 2** (6x6m): east of Room 1, 1.5m doorway at y=0
- **Room 3** (6x6m): south of Room 1, 1.5m doorway at x=0
- Small obstacles in Rooms 2 and 3

## Autonomous exploration

Autonomy is auto-enabled via `run.sh`. The frontier explorer:
1. Finds frontier cells (free space adjacent to unknown) in the SLAM map
2. Clusters them and picks the nearest **safe** cell (away from walls)
3. Sends `NavigateToPose` goals to Nav2
4. Handles aborts with costmap clearing and an unstick (reverse) mechanism
5. Announces `EXPLORATION COMPLETE` when no frontiers remain for 15 consecutive ticks

### Manual override

`cmd_vel_mux` gates between manual and autonomous control:
- `/cmd_vel_manual`: manual input
- `/cmd_vel_nav`: Nav2 output
- `/cmd_vel`: final to Gazebo
- `/autonomy_enabled` (`Bool`): mode switch

Override via web UI or CLI:

```bash
ros2 topic pub /autonomy_enabled std_msgs/msg/Bool "{data: false}" -r 2
ros2 topic pub /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 10
```

## Odometry sources

| Source | Flag | Notes |
|---|---|---|
| Wheel (Gazebo GT) | `odom_source:=wheel` | Recommended for simulation |
| rf2o scan matching | `odom_source:=scan` | Can drift in featureless areas |
| Laser scan matcher | `odom_source:=scan_matcher` | Experimental |

Active odom source publishes `minidog/odom -> minidog/base_footprint` TF. SLAM publishes `map -> minidog/odom`.

## RViz

`rviz/robot.rviz` includes Nav2 visualization: goal pose, global/local plans, costmaps, footprint.
