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
- Sim: Gazebo OdometryPublisher uses `robot_base_frame: base_link`
- Bag: data_relay publishes `odom → base_link` TF directly
- SLAM publishes `map → odom` TF

### System overview

```
┌─────────────────────────────────────────────────────────────┐
│ DATA SOURCE (swappable)                                     │
│                                                             │
│ SIM MODE:                    BAG MODE:                      │
│   Gazebo Fortress              rosbag play                  │
│     │                            │                          │
│   ros_gz_bridge                data_relay.py                │
│     │                            │ (90° odom, 180° scan,    │
│     ├─ /scan                     │  self-ref TF filter,     │
│     ├─ /wheel_odom               │  PointCloud2 relay)      │
│     ├─ /clock                    │                          │
│     └─ /joint_states             ├─ /scan                   │
│                                  ├─ /odom                   │
│ RF2O pipeline (sim only):       ├─ /points                 │
│   /scan → scan_filter            ├─ /tf                     │
│     → /scan_safe → RF2O         └─ /clock                  │
│       → /odom_rf2o                                          │
│         → odom_stabilizer                                   │
│           → /odom + TF(odom→base_link)                     │
│                                                             │
│ RSP (sim only):                                             │
│   base_link → wheels, lidar TF from URDF                   │
│ static_transform_publisher (sim only):                      │
│   base_link → diffbot/base_link/ouster                     │
└─────────────────────────────────────────────────────────────┘
                          │
                    /scan, /odom, TF
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
| 1 | 0s | Gazebo/bag + bridge/relay + odometry pipeline |
| 2 | 0.5s | scan_filter + RF2O (sim only) |
| 3 | 1.0s | odom_stabilizer (sim only) |
| 4 | 1.5s | RViz + cmd_vel_mux |
| 5 | 5.0s | SLAM |
| 6 | 10.0s | Nav2 |
| 7 | 20.0s | Frontier explorer + web UI + rqt |

### Odometry

Three modes via `odom_source` launch argument:

- **`rf2o`** (default): Laser-based odometry — best for both simulation and real hardware.
  Pipeline: `/scan` → `scan_safety_filter` → `/scan_safe` → `rf2o_laser_odometry` → `/odom_rf2o` → `odom_stabilizer` → `/odom` (20Hz) + TF `odom → base_link`.
  - `scan_safety_filter` drops scans with zero time increment to prevent RF2O div-by-zero
  - `rf2o_laser_odometry` runs at 20Hz with `publish_tf: false` (TF delegated to stabilizer)
  - `odom_stabilizer` republishes at a fixed 20Hz timer rate, smoothing jitter. Forces z=0 in TF. Detects staleness (>0.5s) and stops publishing to let TF age naturally
- **`wheel`**: Gazebo ground truth via bridge. Publishes at ~80Hz which can cause RViz jitter. Always available on `/wheel_odom` regardless of mode.
- **`scan_matcher`**: external laser_scan_matcher node. Experimental.

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
- Key tuning: `throttle_scans: 5`
- Config: `config/slam_toolbox_async.yaml`

### Nav2

Two configurations:
- `config/nav2_diffbot.yaml` — differential drive with Spin recovery, rotate-to-heading goal checker
- `config/nav2_slam.yaml` — Ackermann with forward-only navigation, no Spin

Common settings:
- **Planner**: `NavfnPlanner` with A*, tolerance 1.0, `allow_unknown: true`
- **Controller**: `RegulatedPurePursuitController`, 0.25 m/s
- **Costmap inflation**: radius 0.25m, cost_scaling_factor 3.0

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
- **Self-referencing TF filter**: drops `base_link → base_link` transforms
- **PointCloud2 relay**: republishes with updated timestamps
- **odom → base_link TF**: publishes from rotated odom data

### Simulation world

`minidog_world.sdf`: three 6×6m rooms connected by 1.5m doorways.
- Physics: 4ms step, 1x real-time factor
- Room 1: spawn, no obstacles
- Room 2: east, box obstacle
- Room 3: south, cylinder obstacle

### Diffbot physics (model.sdf)

- Wheel friction: mu=100, mu2=100 (prevents lateral sliding)
- Drive wheels at x=0.05 (ahead of CoM) to prevent forward tipping
- Caster friction: mu=0.001 (slides freely)
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
| `minidog_sim/data_relay.py` | GO2 rosbag adapter |
| `minidog_sim/odom_stabilizer.py` | RF2O output smoother (20Hz) |
| `minidog_sim/scan_filter.py` | Zero-increment scan filter |
| `urdf/diffbot.urdf.xacro` | Differential drive URDF |
| `models/diffbot/model.sdf` | Gazebo SDF model |
| `rviz/robot.rviz` | Unified RViz config |
| `config/slam_toolbox_async.yaml` | SLAM tuning |
| `config/nav2_diffbot.yaml` | Nav2 diff drive config |
