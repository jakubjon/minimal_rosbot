# Developer Notes

Knowledge base for continuing development across sessions. This file captures conventions, known issues, and implementation details not covered in README.md or ARCHITECTURE.md.

## Project Context

The ROBOPES repo contains three projects:
- **minimal_rosbot/** — unified sim/real exploration pipeline (this project)
- **qre_go2/** — Unitree GO2 commercial SDK by MYBOTSHOP (ROS2 Foxy)
- **VUGR/** — VUT/G4D simulation & SLAM testing framework

The real robot is a **Unitree GO2-W** (namespace `go2_unit_27778`) that behaves like a differential drive (forward/backward + rotate in place). Its chin lidar is broken, so an **Ouster** lidar + **RF2O** laser odometry provides positioning (via VUGR/dupak container on the robot).

## Rosbag

Located at `~/Desktop/20260115_3`. Duration: ~95s.

| Topic | Count | Rate | Notes |
|---|---|---|---|
| `/go2_unit_27778/ouster/scan` | 628 | ~6.6Hz | LaserScan |
| `/go2_unit_27778/ouster/points` | 154 | ~1.6Hz | PointCloud2 |
| `/go2_unit_27778/base/odom` | 931 | ~10Hz | RF2O odom (already computed) |
| `/go2_unit_27778/sensor/imu/data` | 8447 | ~100Hz | Body IMU (go2_statepublisher) |
| `/go2_unit_27778/ouster/imu` | 8453 | ~100Hz | Ouster internal IMU (not used) |
| `/go2_unit_27778/sensor/camera_raw/compressed` | 1316 | — | Not relayed |

**Body IMU quirks** (go2_statepublisher): `frame_id=""` (empty), all covariance arrays = 0. Fixed by `data_relay.py`.
**Ouster IMU** (`ouster/imu`) is not used — Ouster is mounted with axes rotated 180°, adding complexity. Body IMU is simpler.

## Unified Frame Convention

```
map → odom → base_link → lidar_link (sim) / os_sensor (bag)
```

- **No `base_footprint`** anywhere in diffbot. `base_link` is the URDF root.
- Gazebo SDF `OdometryPublisher` uses `robot_base_frame: base_link`
- All launch files, nodes, and configs use `base_link` consistently
- The minidog/ackermann model still uses `base_footprint` (legacy, not unified yet)

## RF2O Odometry Pipeline

Both modes (`odom_source:=rf2o`):

```
/scan (10Hz sim / 6.6Hz bag)
  → scan_safety_filter (drops zero time_increment scans)
    → /scan_safe
      → rf2o_laser_odometry (publish_tf: false)
        → /odom_rf2o
          → odom_stabilizer (20Hz timer, publish_tf: false when EKF on)
            → /odom
```

Key details:
- `odom_stabilizer` forces z=0 in TF to prevent vertical drift
- Detects staleness (>0.5s no new data) and stops publishing
- Wheel odom always available on `/wheel_odom` (~80Hz) for debugging

## EKF Fusion (enable_ekf:=true, default)

`robot_localization` EKF node fuses RF2O odom + body IMU gyro:

```
/odom (20Hz RF2O via stabilizer) ──┐
                                    ├─ ekf_node → /odom/filtered (~30Hz) + TF odom→base_link
/imu/data (100Hz gyro yaw-rate) ───┘
```

Config `config/ekf.yaml`:
- `two_d_mode: true` — ground robot, ignores z/roll/pitch
- `odom0: /odom` — fuses x, y, yaw, vx, vyaw
- `imu0: /imu/data` — fuses **only** angular_velocity.z (gyro yaw-rate)
  - `imu0_config[11]=true`, all others false
  - Linear acceleration excluded (legged robot leg-oscillation noise)
- `publish_tf: true` — EKF owns `odom → base_link` TF
- `frequency: 30.0`

When `enable_ekf:=true`, `odom_stabilizer` sets `publish_tf: false` to avoid TF conflict. Nav2 `bt_navigator` receives `odom_topic: /odom/filtered`.

`enable_ekf:=false` fallback: odom_stabilizer publishes `/odom` + `odom→base_link` TF at 20Hz.

## IMU Pipeline

### Simulation

1. Gazebo IMU sensor in `model.sdf` publishes to `/diffbot/imu` at 100Hz
   - Requires `ignition-gazebo-imu-system` plugin in `minidog_world.sdf`
   - Gaussian noise: σ=0.002 rad/s (gyro), σ=0.04 m/s² (accel)
2. `ros_gz_bridge` maps `/diffbot/imu` → `/imu/bridge_raw`
   - Sets `frame_id` to Gazebo scoped name (`diffbot/base_link/imu_sensor`) — **wrong**
   - Leaves all covariances = 0 — **wrong for EKF**
3. `imu_fixup.py` subscribes to `/imu/bridge_raw`, fixes both issues, publishes `/imu/data`

### Bag

1. `go2_statepublisher` publishes `/{ns}/sensor/imu/data` at ~100Hz
   - `frame_id=""`, all covariances = 0 — **wrong for EKF**
2. `data_relay.py` fixes both issues, publishes `/imu/data`

### Covariance convention (σ², not σ)

ROS `sensor_msgs/Imu` covariance fields must contain **variances** (σ²), not standard deviations:

| Field | σ | σ² |
|---|---|---|
| orientation xy | 0.1 rad | 0.01 rad² |
| orientation z (yaw) | 0.316 rad | 0.1 rad² |
| angular_velocity | 0.002 rad/s | 4e-6 rad²/s² |
| linear_acceleration | 0.04 m/s² | 1.6e-3 m²/s⁴ |

Both `imu_fixup.py` and `data_relay.py` use the same σ² values so EKF weights are identical across modes.

## Data Relay (bag mode)

`data_relay.py` handles GO2-W rosbag quirks:
- **90° rotation (position + orientation + velocity)**: GO2 X=left, Y=forward → standard ROS X=forward via -90° Z rotation applied as quaternion pre-multiply `q_rot⊗q_orig`
- **180° scan rotation**: Ouster mounted backward, ranges array shifted by half
- **Self-referencing TF filter**: drops `os_sensor → os_sensor` (180° yaw hack) from bag
- **IMU relay**: fixes `frame_id=""` → `base_link`, injects σ² covariances
- **PointCloud2 relay**: deep-copies with updated timestamps
- **`relay_odom` parameter**: when `false` (default in rf2o mode), skips odom subscription so RF2O owns the odom/TF output

## RF2O in bag mode (default, odom_source:=rf2o)

Instead of replaying the recorded GO2 odom, RF2O runs fresh on the corrected scan:
```
bag scan → data_relay (180° rotate) → /scan → scan_filter → RF2O → odom_stabilizer → /odom
                                                                                          ↓
bag IMU  → data_relay (frame fix)   → /imu/data ──────────────────────────────── EKF → /odom/filtered
```
Benefits: consistent coordinate frame, 20Hz smooth odom, eliminates recorded-odom drift, EKF fusion available.
To revert to recorded odom: `odom_source:=wheel` (data_relay.relay_odom stays true).

## Sim Fidelity (matched to real GO2-W hardware)

Changes made to bring sim closer to real robot for better exploration tuning transfer:

| Parameter | Before | After | Reason |
|---|---|---|---|
| Lidar max range | 20m | 10m | Real Ouster range in use |
| Lidar noise | none | Gaussian σ=0.015m | Real Ouster measurement noise |
| SLAM max_laser_range | 20m | 10m | Matches lidar hardware range |
| rotate_to_heading_angular_vel | 1.2 rad/s | 0.8 rad/s | Real GO2-W spins at ~0.6-0.9 rad/s |
| IMU sensor | none | 100Hz, matched noise | Real GO2-W body IMU |

## Diffbot Physics (model.sdf)

- Wheel friction: mu=100, mu2=100 (prevents lateral sliding)
- Drive wheels at x=0.05 (slightly ahead of CoM at x=0.005) to prevent forward tipping
- Caster friction: mu=0.001 (slides freely)
- Chassis: 0.50×0.30×0.10m, 5kg
- Lidar: 360×16 beams, 10Hz, 0.1–10m, Gaussian σ=0.015m
- IMU: 100Hz collocated with lidar, Gaussian σ=0.002 rad/s (gyro), σ=0.04 m/s² (accel)

## Nav2 Configurations

- `nav2_diffbot.yaml` — differential drive: Spin recovery enabled, rotate-to-heading goal checker, `rotate_to_heading_angular_vel: 0.8` rad/s (tuned for GO2-W)
- `nav2_slam.yaml` — Ackermann: forward-only, no Spin, relaxed yaw tolerance (6.28 rad)
- Common: NavfnPlanner A*, RegulatedPurePursuit 0.25 m/s, inflation radius 0.25m
- `odom_topic`: forwarded from bringup as `/odom/filtered` (EKF on) or `/odom` (EKF off)

## Frontier Explorer

- Heading-aware scoring: `(distance / sqrt(size)) * heading_penalty`
- `heading_weight`: 0.5 for diffbot (can spin), 2.0 for ackermann (forward-biased)
- Blocked goal blacklist: 1.5m radius, 90s TTL
- Unstick: reverse 5s at 0.20 m/s + clear costmaps
- Completion: 8 ticks (~16s) with no frontiers

## DDS Configuration

`config/fastdds_no_shm.xml` disables shared-memory transport (UDP only).
- **Required on WSL2** (shared memory causes hangs)
- **Optional on native Linux** (harmless to keep)
- Set via `FASTRTPS_DEFAULT_PROFILES_FILE` env var in bringup.launch.py

## Known Issues / Gotchas

1. **Gazebo startup on WSL2**: Takes 30-45s. RF2O will spam "Waiting for laser_scans" until Gazebo produces data. This is normal.
2. **Stale DDS entries**: After killing processes, `ros2 topic info` may show ghost publishers. Clean with `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` and wait a few seconds.
3. **SLAM queue full**: On bag replay startup, slam_toolbox drops messages ("discarding message because the queue is full"). Normal — it catches up after a few seconds.
4. **Bag replay `Message queue starved`**: rosbag player warns about queue starvation when disk I/O is slow. Not an issue on native Linux with bag on local disk.
5. **`data_relay.py` must be executable**: symlink-install requires `chmod +x` on Python scripts. If `run_bag.sh` fails with "executable not found", run `chmod +x src/minidog_sim/minidog_sim/data_relay.py` and rebuild.
6. **Cleanup kills launch process**: The cleanup step in bringup.launch.py `pkill -9` can sometimes kill itself. This is expected — the `OnProcessExit` handler then starts the actual nodes.
7. **Gazebo IMU requires `ignition-gazebo-imu-system`**: The `Sensors` plugin (for lidar/camera) does NOT handle IMU. IMU sensors in Gazebo Fortress need the separate `ignition-gazebo-imu-system` plugin loaded in the world SDF. Without it, `/diffbot/imu` is never published.
8. **ros_gz_bridge sets wrong IMU frame_id**: The bridge sets `frame_id` to the Gazebo scoped sensor name (e.g., `diffbot/base_link/imu_sensor`) instead of `base_link`. This would make EKF fail to find the TF. `imu_fixup.py` corrects this.
9. **PythonExpression yields string for bool params**: When passing `PythonExpression(["'false'"])` as a ROS 2 parameter declared as `bool`, the node receives a string `'false'` which is truthy in Python. `odom_stabilizer.py` handles this by checking `isinstance(val, bool)` and converting strings explicitly.
10. **`robot_localization` must be installed**: `sudo apt-get install -y ros-humble-robot-localization`. Not installed by default with ROS Humble.

## qre_go2 Reference (for visualization style)

The RViz visualization style (dark background, grouped displays, orange/lime plans) is inspired by qre_go2's configs. Key differences:
- qre_go2 uses CycloneDDS, ROS_DOMAIN_ID=10, namespace isolation
- qre_go2 targets ROS2 Foxy on NVIDIA Orin NX
- qre_go2 has full hardware control (gaits, arms, cameras) — minimal_rosbot only does navigation

## VUGR Reference (for bag replay)

VUGR/dupak contains the RF2O configuration used on the real GO2-W:
- `rf2o_laser_odometry.launch.py` — subscribes to `/go2_unit_27778/ouster/scan`
- The recorded odom in the bag comes from this RF2O instance
- VUGR's `data_relay_node.py` and `tf_relay_node.py` inspired minimal_rosbot's `data_relay.py`
