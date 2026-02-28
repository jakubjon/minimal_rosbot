# minimal_rosbot (ROS 2 Humble + Gazebo Fortress)

Unified simulation and real-robot pipeline for autonomous frontier exploration.
Supports two data sources — **Gazebo simulation** and **rosbag replay** (real GO2-W) — with identical frame conventions, topics, and visualization.

Stack: **ROS 2 Humble**, **Gazebo Fortress (Ignition 6)**, **slam_toolbox**, **Nav2**, **Streamlit web UI**.

## Quick start

```bash
# Simulation (Gazebo + SLAM + Nav2 + exploration + RViz)
bash run_sim.sh

# Rosbag replay (real GO2-W data + SLAM + RViz)
bash run_bag.sh
bash run_bag.sh /path/to/bag              # custom bag path
bash run_bag.sh /path/to/bag my_namespace  # custom bag + namespace
```

Both scripts kill zombie processes, clean DDS shared memory, build the workspace, and launch with RViz.

Web UI (sim mode): `http://localhost:8501`

## Build

```bash
cd minimal_rosbot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Data sources

### Simulation (`data_source:=sim`)

Launches Gazebo with a differential-drive robot (`diffbot`) in a 3-room arena.
RF2O laser odometry provides smooth 20Hz odom. Full autonomous pipeline available (SLAM + Nav2 + frontier explorer).

### Bag replay (`data_source:=bag`)

Replays a real GO2-W rosbag through the same pipeline. The `data_relay` node handles GO2-specific quirks:
- 90° odom rotation (GO2 forward = ROS Y-axis)
- 180° scan rotation (Ouster mounted backward)
- Self-referencing TF filter (drops `base_link → base_link`)
- Relays: `/scan`, `/odom`, `/points` (PointCloud2), `/tf`

RF2O odom is already recorded in the bag — no need to re-run it.

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `data_source` | `sim` | `sim` (Gazebo) or `bag` (rosbag replay) |
| `robot_type` | `diffbot` | `diffbot` (diff drive) or `ackermann` (minidog) |
| `odom_source` | `rf2o` | `rf2o`, `wheel` (Gazebo GT), `scan`, `scan_matcher` |
| `bag_path` | `/mnt/c/.../20260115_3` | Path to rosbag directory |
| `bag_namespace` | `go2_unit_27778` | GO2 namespace in rosbag |
| `enable_slam` | `true` | Start slam_toolbox |
| `enable_nav2` | `false` | Start Nav2 stack |
| `enable_explore` | `false` | Start frontier explorer |
| `enable_mux` | `true` | Start cmd_vel multiplexer |
| `enable_rviz` | `true` | Start RViz |
| `enable_web` | `false` | Start Streamlit web UI |
| `headless` | `false` | Gazebo without GUI |
| `quiet_terminal` | `true` | Route output to log files |

## Frame conventions (unified)

Both sim and bag modes use identical frames:

```
map → odom → base_link → lidar_link (sim) / os_sensor (bag)
```

- No `base_footprint` — `base_link` is the root frame
- `/odom` topic: `frame_id: odom`, `child_frame_id: base_link`
- SLAM publishes `map → odom` TF
- Odom source publishes `odom → base_link` TF

## Topics (unified)

| Topic | Type | Source (sim) | Source (bag) |
|---|---|---|---|
| `/scan` | LaserScan | Gazebo bridge | data_relay |
| `/odom` | Odometry | odom_stabilizer | data_relay |
| `/points` | PointCloud2 | Gazebo bridge | data_relay |
| `/map` | OccupancyGrid | slam_toolbox | slam_toolbox |
| `/tf` | TF | RSP + odom_stabilizer | data_relay |
| `/wheel_odom` | Odometry | Gazebo bridge | — |

## Odometry

| Source | Flag | Notes |
|---|---|---|
| RF2O (default) | `odom_source:=rf2o` | Laser-based, 20Hz via odom_stabilizer. Best for sim + real |
| Wheel (Gazebo GT) | `odom_source:=wheel` | ~80Hz, can cause RViz jitter. Always on `/wheel_odom` |
| Scan matcher | `odom_source:=scan_matcher` | Experimental |

RF2O pipeline: `/scan` → `scan_filter` → `/scan_safe` → RF2O → `/odom_rf2o` → `odom_stabilizer` → `/odom` + TF

## RViz

Single unified config (`rviz/robot.rviz`) for both sim and bag modes:
- **Robot group**: RobotModel (sim), TF frames, odometry arrows
- **Sensors group**: LaserScan (white boxes), PointCloud2 (intensity rainbow, 3s decay)
- **Navigation group**: SLAM map, costmaps, global/local plans, goal pose
- Dark background, TopDownOrtho view

## Staggered startup

Components start in phases after cleanup (kills zombies from previous runs):

| Phase | Delay | Components |
|---|---|---|
| 1 | 0s | Gazebo/bag + bridge/relay + odometry |
| 2 | 1.5s | RViz + cmd_vel_mux |
| 3 | 5s | SLAM |
| 4 | 10s | Nav2 |
| 5 | 20s | Frontier explorer + web UI |

## Autonomous exploration

The frontier explorer (enabled via `run_sim.sh`):
1. Finds frontier cells in the SLAM map
2. Clusters via 8-connected BFS, scores by distance/size with heading penalty
3. Sends NavigateToPose goals to Nav2
4. On abort: reverses 5s, clears costmaps
5. Announces `EXPLORATION COMPLETE` when no frontiers remain

### Manual override

```bash
ros2 topic pub /autonomy_enabled std_msgs/msg/Bool "{data: false}" -r 2
ros2 topic pub /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 10
```

## Platform notes

- **Linux** (recommended): native Gazebo/RViz performance
- **WSL2**: works but Gazebo startup is slow (~30-45s). FastDDS configured for UDP-only (no shared memory) via `config/fastdds_no_shm.xml`
