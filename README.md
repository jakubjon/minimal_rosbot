# minimal_rosbot (ROS 2 Humble + Gazebo Fortress)

Unified simulation and real-robot pipeline for autonomous frontier exploration.
Supports two data sources — **Gazebo simulation** and **rosbag replay** (real GO2-W) — with identical frame conventions, topics, and visualization.

Stack: **ROS 2 Humble**, **Gazebo Fortress (Ignition 6)**, **slam_toolbox**, **Nav2**, **robot_localization EKF**, **Streamlit web UI**.

## Quick start

```bash
# Simulation (Gazebo + RF2O + EKF + SLAM + Nav2 + exploration + RViz)
bash run_sim.sh

# Rosbag replay (real GO2-W data + RF2O + EKF + SLAM + RViz)
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

Required apt packages:
```bash
sudo apt-get install -y ros-humble-robot-localization
```

## Data sources

### Simulation (`data_source:=sim`)

Launches Gazebo with a differential-drive robot (`diffbot`) in a 3-room arena.
RF2O laser odometry provides smooth 20Hz odom. IMU sensor simulates the GO2-W body IMU at 100Hz with Gaussian noise matching real hardware.
Full autonomous pipeline available (RF2O → EKF → SLAM → Nav2 → frontier explorer).

Sim lidar specs (matched to real Ouster): 10m max range, 0.015m Gaussian noise, 360×16 beams at 10Hz.

### Bag replay (`data_source:=bag`)

Replays a real GO2-W rosbag through the same pipeline. The `data_relay` node handles GO2-specific quirks:
- 90° odom rotation (GO2 forward = ROS Y-axis)
- 180° scan rotation (Ouster mounted backward)
- Self-referencing TF filter (drops `os_sensor → os_sensor`)
- IMU relay: fixes empty `frame_id` and zero covariances from `go2_statepublisher`
- Relays: `/scan`, `/imu/data`, `/points` (PointCloud2), `/tf`

RF2O runs fresh on the corrected scan (not the recorded odom), producing smooth 20Hz odom. EKF fuses RF2O + body IMU gyro for smoother rotation tracking.

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `data_source` | `sim` | `sim` (Gazebo) or `bag` (rosbag replay) |
| `robot_type` | `diffbot` | `diffbot` (diff drive) or `ackermann` (minidog) |
| `odom_source` | `rf2o` | `rf2o` (default), `wheel` (Gazebo GT) |
| `bag_path` | `~/Desktop/20260115_3` | Path to rosbag directory |
| `bag_namespace` | `go2_unit_27778` | GO2 namespace in rosbag |
| `enable_ekf` | `true` | Fuse RF2O odom + IMU gyro via robot_localization EKF |
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
- EKF publishes `odom → base_link` TF (owns the TF when `enable_ekf:=true`)

## Topics (unified)

| Topic | Type | Source (sim) | Source (bag) |
|---|---|---|---|
| `/scan` | LaserScan | Gazebo bridge | data_relay |
| `/imu/data` | Imu | imu_fixup (bridge raw → fixed) | data_relay |
| `/odom` | Odometry | odom_stabilizer | odom_stabilizer |
| `/odom/filtered` | Odometry | EKF (RF2O + IMU) | EKF (RF2O + IMU) |
| `/points` | PointCloud2 | Gazebo bridge | data_relay |
| `/map` | OccupancyGrid | slam_toolbox | slam_toolbox |
| `/tf` | TF | RSP + EKF | data_relay + EKF |
| `/wheel_odom` | Odometry | Gazebo bridge | — |

## Odometry pipeline

RF2O + EKF fusion (default, `odom_source:=rf2o`, `enable_ekf:=true`):

```
/scan → scan_filter → /scan_safe → RF2O → /odom_rf2o
                                               ↓
                                     odom_stabilizer → /odom (20Hz, no TF)
                                               ↓
    /imu/data (100Hz gyro) ──────── EKF node → /odom/filtered (~30Hz) + TF odom→base_link
```

- **odom_stabilizer**: smooths RF2O jitter, detects staleness. `publish_tf: false` when EKF enabled (EKF owns `odom → base_link` TF).
- **EKF**: fuses pose/twist from `/odom` + gyro yaw-rate from `/imu/data`. `two_d_mode: true`, 30Hz output. Config: `config/ekf.yaml`.
- Nav2 `bt_navigator` subscribes to `/odom/filtered` when EKF is enabled.

Fallback without EKF (`enable_ekf:=false`): odom_stabilizer publishes `/odom` + `odom→base_link` TF at 20Hz.

| Source | Flag | Notes |
|---|---|---|
| RF2O + EKF (default) | `odom_source:=rf2o enable_ekf:=true` | 30Hz filtered odom, gyro-fused rotation |
| RF2O only | `odom_source:=rf2o enable_ekf:=false` | 20Hz, no IMU fusion |
| Wheel (Gazebo GT) | `odom_source:=wheel` | ~80Hz, sim only, always on `/wheel_odom` |

## IMU

| Mode | Source | Processing |
|---|---|---|
| Sim | Gazebo IMU sensor → bridge → `/imu/bridge_raw` | `imu_fixup.py`: sets `frame_id=base_link`, injects σ² covariances |
| Bag | `/{ns}/sensor/imu/data` (100Hz body IMU) | `data_relay.py`: sets `frame_id=base_link`, injects σ² covariances |

EKF fuses only angular velocity Z (gyro yaw-rate). Linear acceleration is not fused (too noisy on legged robot with leg oscillation).

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
| 0 | — | Cleanup (kill zombies, clean DDS shm) |
| 1 | 0s | Gazebo/bag + bridge/relay |
| 2a | 0.5s | scan_filter + RF2O + imu_fixup (sim) |
| 2b | 1.0s | odom_stabilizer |
| 3 | 1.5s | RViz + cmd_vel_mux + EKF |
| 4 | 5s | SLAM |
| 5 | 10s | Nav2 |
| 6 | 20s | Frontier explorer + web UI |

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

- **Linux** (native, recommended): full performance, ogre2 renderer, shared memory DDS optional
- **WSL2**: Gazebo startup slow (~30-45s). FastDDS configured for UDP-only (no shared memory) via `config/fastdds_no_shm.xml`
