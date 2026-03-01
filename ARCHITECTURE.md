## minidog_sim architecture

### Design intent

- **Unified sim/real pipeline**: same frame IDs, topics, and visualization for both Gazebo simulation and real robot rosbag replay. Swap `data_source:=sim` / `data_source:=bag` with no other changes.
- **Sandbox for autonomous exploration**: SLAM + Nav2 + frontier exploration, reusable patterns for real hardware.
- **Two robot models**: differential drive (`diffbot`, default — matches real GO2-W) and Ackermann (`minidog`, legacy).
- **Two data sources**: Gazebo simulation (`data_source:=sim`) or real GO2-W rosbag replay (`data_source:=bag`).

### Frame convention (unified)

```
map → odom → base_link → lidar_link (sim) / os_sensor (bag)
```

- `base_link` is the root frame in both modes (no `base_footprint`)
- EKF publishes `odom → base_link` TF when `enable_ekf:=true` (odom_stabilizer delegates TF to EKF)
- SLAM publishes `map → odom` TF

### System overview

```
┌─────────────────────────────────────────────────────────────────────┐
│ DATA SOURCE (swappable)                                             │
│                                                                     │
│ SIM MODE:                          BAG MODE:                        │
│   Gazebo Fortress                    rosbag play                    │
│     │                                   │                           │
│   ros_gz_bridge                      data_relay.py                  │
│     │                                   │ (90° odom, 180° scan,     │
│     ├─ /scan                            │  self-ref TF filter,      │
│     ├─ /imu/bridge_raw                  │  IMU frame+cov fix,       │
│     ├─ /wheel_odom                      │  PointCloud2 relay)       │
│     ├─ /clock                           │                           │
│     └─ /joint_states                    ├─ /scan                    │
│                                         ├─ /imu/data (100Hz)        │
│   imu_fixup.py:                         ├─ /odom (if relay_odom)    │
│     /imu/bridge_raw                     ├─ /points                  │
│       → /imu/data (100Hz)               ├─ /tf                      │
│       (frame_id=base_link,              └─ /clock                   │
│        σ² covariances)                                              │
│                                                                     │
│ RF2O pipeline (both modes when odom_source:=rf2o):                 │
│   /scan → scan_filter → /scan_safe                                  │
│     → rf2o_laser_odometry → /odom_rf2o                              │
│       → odom_stabilizer → /odom (20Hz, publish_tf=false w/ EKF)    │
│                                                                     │
│ EKF (both modes when enable_ekf:=true):                             │
│   /odom + /imu/data → ekf_node → /odom/filtered (~30Hz)            │
│                               → TF odom→base_link                  │
│                                                                     │
│ RSP (sim only):                                                     │
│   base_link → wheels, lidar TF from URDF                           │
└─────────────────────────────────────────────────────────────────────┘
                          │
               /scan, /odom/filtered, TF
                          │
                          ▼
              ┌───────────────────────┐
              │ SHARED PIPELINE       │
              │                       │
              │ slam_toolbox          │
              │   map→odom TF, /map   │
              │                       │
              │ Nav2                  │
              │   /cmd_vel_nav        │
              │   (odom: /odom/filtered│
              │    when EKF enabled)  │
              │                       │
              │ frontier_explorer     │
              │   NavigateToPose      │
              │                       │
              │ cmd_vel_mux           │
              │   /cmd_vel            │
              │                       │
              │ RViz (robot.rviz)     │
              │ Streamlit web UI      │
              └───────────────────────┘
```

### Staggered launch order

Race conditions are avoided with `TimerAction` delays in `bringup.launch.py`:

| Phase | Delay | Components |
|---|---|---|
| 0 | — | Cleanup (kill zombies from previous runs) |
| 1 | 0s | Gazebo/bag + bridge/relay |
| 2a | 0.5s | scan_filter + RF2O + imu_fixup (sim) |
| 2b | 1.0s | odom_stabilizer |
| 3 | 1.5s | RViz + cmd_vel_mux + EKF |
| 4 | 5.0s | SLAM |
| 5 | 10.0s | Nav2 |
| 6 | 20.0s | Frontier explorer + web UI + rqt |

### Odometry

RF2O + EKF fusion (`odom_source:=rf2o`, `enable_ekf:=true`, both modes):

```
/scan → scan_filter → /scan_safe → rf2o_laser_odometry → /odom_rf2o
                                                              ↓
                                               odom_stabilizer → /odom (20Hz)
                                                              ↓
    /imu/data (100Hz gyro) ─────────── ekf_node → /odom/filtered (~30Hz) + TF
```

- `scan_safety_filter` drops scans with zero time increment to prevent RF2O div-by-zero
- `rf2o_laser_odometry` runs at 20Hz with `publish_tf: false` (TF delegated to EKF)
- `odom_stabilizer`: republishes at 20Hz timer, forces z=0 in TF, detects staleness (>0.5s). Sets `publish_tf: false` when EKF is enabled — EKF owns `odom → base_link` TF
- **EKF** (`robot_localization`): 2D mode, 30Hz. Fuses RF2O pose/twist (x, y, yaw, vx, vyaw) + IMU gyro yaw-rate only. Config: `config/ekf.yaml`
- Wheel odom always available on `/wheel_odom` regardless of mode

Without EKF (`enable_ekf:=false`): odom_stabilizer publishes `/odom` + `odom→base_link` TF at 20Hz.

### IMU

| Mode | Raw source | Processing | Output |
|---|---|---|---|
| Sim | Gazebo IMU sensor (`ignition-gazebo-imu-system` required) → bridge `/imu/bridge_raw` | `imu_fixup.py`: sets `frame_id=base_link`, injects σ² covariances | `/imu/data` 100Hz |
| Bag | `/{ns}/sensor/imu/data` (go2_statepublisher, 100Hz) | `data_relay.py`: sets `frame_id=base_link`, injects σ² covariances | `/imu/data` ~100Hz |

EKF fuses only `angular_velocity.z` (gyro yaw-rate, `imu0_config[11]=true`). Linear acceleration excluded (legged robot vibration).

Covariance values (σ², not σ — both modes):
- Orientation: diag(0.01, 0.01, 0.1) rad²
- Gyro: diag(4e-6, 4e-6, 4e-6) rad²/s²
- Accel: diag(1.6e-3, 1.6e-3, 1.6e-3) m²/s⁴

### Commanding (manual vs autonomy)

`minidog_cmd_mux` multiplexes velocity commands:
- `/cmd_vel_manual` (manual joystick/web UI)
- `/cmd_vel_nav` (Nav2 output)
- `/cmd_vel` (final output to Gazebo)
- `/autonomy_enabled` (`Bool`): when true, forwards `/cmd_vel_nav`; when false, forwards `/cmd_vel_manual`

### SLAM

`slam_toolbox` (async mode):
- Subscribes: `/scan`, TF
- Publishes: `/map`, `map → odom` TF
- Key tuning: `throttle_scans: 5`, `max_laser_range: 10.0` (matched to lidar hardware range)
- Config: `config/slam_toolbox_async.yaml`

### Nav2

Two configurations:
- `config/nav2_diffbot.yaml` — differential drive with Spin recovery, rotate-to-heading goal checker, `rotate_to_heading_angular_vel: 0.8` rad/s (matched to real GO2-W)
- `config/nav2_slam.yaml` — Ackermann with forward-only navigation, no Spin

Common settings:
- **Planner**: `NavfnPlanner` with A*, tolerance 1.0, `allow_unknown: true`
- **Controller**: `RegulatedPurePursuitController`, 0.25 m/s
- **Costmap inflation**: radius 0.25m, cost_scaling_factor 3.0
- **odom_topic**: `/odom/filtered` when EKF enabled, `/odom` otherwise (bt_navigator parameter)

### Frontier explorer

`minidog_frontier_explorer` (`frontier_explore.py`):

**Goal selection** (heading-aware, forward-biased):
1. Find frontier cells (free cells adjacent to unknown) in `/map`
2. Cluster via 8-connected BFS, discard clusters < 5 cells
3. Score: `(distance / sqrt(cluster_size)) * heading_penalty`
4. heading_penalty favors goals ahead of robot (configurable weight)

**Robustness**: blocked goal blacklist (1.5m radius, 90s TTL), immediate unstick on abort (reverse 5s + clear costmaps), wall-time timeout (30s), completion detection (8 ticks with no frontiers).

### Data relay (bag mode)

`data_relay.py` handles GO2-W rosbag quirks:
- **90° odom rotation**: GO2 forward = ROS Y-axis → rotated to standard ROS X-forward
- **180° scan rotation**: Ouster mounted backward → angles shifted by π
- **Self-referencing TF filter**: drops `os_sensor → os_sensor` (180° yaw hack) from bag
- **IMU relay**: fixes `frame_id=""` and zero covariances from go2_statepublisher; injects σ² covariances
- **PointCloud2 relay**: republishes with updated timestamps
- **`relay_odom` parameter**: `false` in rf2o mode (RF2O owns odom/TF), `true` if replaying recorded odom

### Simulation world

`minidog_world.sdf`: three 6×6m rooms connected by 1.5m doorways.
- Physics: 4ms step, 1x real-time factor
- Required plugins: `ignition-gazebo-physics-system`, `ignition-gazebo-sensors-system`, `ignition-gazebo-imu-system`, `ignition-gazebo-scene-broadcaster-system`
- Room 1: spawn, no obstacles
- Room 2: east, box obstacle
- Room 3: south, cylinder obstacle

### Diffbot model (model.sdf)

- Wheel friction: mu=100, mu2=100 (prevents lateral sliding)
- Drive wheels at x=0.05 (ahead of CoM) to prevent forward tipping
- Caster friction: mu=0.001 (slides freely)
- **Lidar**: 360×16 beams, 10Hz, 0.1–10m range (matched to real Ouster), Gaussian noise σ=0.015m
- **IMU sensor**: 100Hz, collocated with lidar, Gaussian noise σ=0.002 rad/s (gyro), σ=0.04 m/s² (accel)
- Gazebo OdometryPublisher: `robot_base_frame: base_link`

### RViz

Single unified config `rviz/robot.rviz` for both sim and bag modes:
- Dark background (61; 56; 70), qre_go2-inspired style
- **Robot group**: RobotModel (shows in sim), TF, Odometry arrows
- **Sensors group**: LaserScan (white boxes), PointCloud2 (intensity rainbow, 3s decay)
- **Navigation group**: Map (draw behind), costmaps, orange global plan, lime local plan
- TopDownOrtho view

### DDS configuration

`config/fastdds_no_shm.xml` disables shared-memory transport (UDP only). Required on WSL2, optional on native Linux.

### Key files

| File | Purpose |
|---|---|
| `run_sim.sh` | Launch simulation (kill zombies + build + full stack) |
| `run_bag.sh` | Launch bag replay (build + SLAM + RViz) |
| `launch/bringup.launch.py` | Main orchestrator (conditions, staggered launch) |
| `launch/rviz.launch.py` | RSP + static TF + RViz |
| `launch/sim.launch.py` | Gazebo world launch |
| `launch/bridge.launch.py` | ros_gz_bridge config |
| `launch/nav2.launch.py` | Nav2 stack (odom_topic forwarded from bringup) |
| `minidog_sim/data_relay.py` | GO2 rosbag adapter (scan, IMU, TF, PointCloud2) |
| `minidog_sim/odom_stabilizer.py` | RF2O output smoother (20Hz) |
| `minidog_sim/scan_filter.py` | Zero-increment scan filter |
| `minidog_sim/imu_fixup.py` | Sim IMU: fix frame_id + inject covariances |
| `urdf/diffbot.urdf.xacro` | Differential drive URDF |
| `models/diffbot/model.sdf` | Gazebo SDF model (lidar noise, IMU sensor) |
| `worlds/minidog_world.sdf` | Simulation arena (3 rooms, IMU system plugin) |
| `rviz/robot.rviz` | Unified RViz config |
| `config/ekf.yaml` | EKF node config (RF2O + IMU gyro fusion) |
| `config/slam_toolbox_async.yaml` | SLAM tuning (max_laser_range: 10m) |
| `config/nav2_diffbot.yaml` | Nav2 diff drive config (rotate_to_heading: 0.8 rad/s) |
