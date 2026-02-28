# Developer Notes

Knowledge base for continuing development across sessions. This file captures conventions, known issues, and implementation details not covered in README.md or ARCHITECTURE.md.

## Project Context

The ROBOPES repo contains three projects:
- **minimal_rosbot/** — unified sim/real exploration pipeline (this project)
- **qre_go2/** — Unitree GO2 commercial SDK by MYBOTSHOP (ROS2 Foxy)
- **VUGR/** — VUT/G4D simulation & SLAM testing framework

The real robot is a **Unitree GO2-W** (namespace `go2_unit_27778`) that behaves like a differential drive (forward/backward + rotate in place). Its chin lidar is broken, so an **Ouster** lidar + **RF2O** laser odometry provides positioning (via VUGR/dupak container on the robot).

## Rosbag

Located at `/mnt/c/Users/Jakub/Desktop/20260115_3` (WSL path — adjust for Linux).
Contents:
- `/go2_unit_27778/base/odom` — 931 msgs (~10Hz, RF2O laser odom already computed)
- `/go2_unit_27778/ouster/scan` — 628 msgs (~6.6Hz, LaserScan)
- `/go2_unit_27778/ouster/points` — 154 msgs (~1.6Hz, PointCloud2)
- `/go2_unit_27778/sensor/camera_raw/compressed` — 1316 msgs
- TF, joint_states, IMU, battery, etc.
- Duration: ~95s

## Unified Frame Convention

```
map → odom → base_link → lidar_link (sim) / os_sensor (bag)
```

- **No `base_footprint`** anywhere in diffbot. `base_link` is the URDF root.
- Gazebo SDF `OdometryPublisher` uses `robot_base_frame: base_link`
- All launch files, nodes, and configs use `base_link` consistently
- The minidog/ackermann model still uses `base_footprint` (legacy, not unified yet)

## RF2O Odometry Pipeline (sim mode)

```
/scan (10Hz from Gazebo)
  → scan_safety_filter (drops zero time_increment scans)
    → /scan_safe
      → rf2o_laser_odometry (20Hz, publish_tf: false)
        → /odom_rf2o
          → odom_stabilizer (20Hz timer, publishes /odom + TF odom→base_link)
```

Key details:
- `odom_stabilizer` forces z=0 in TF to prevent vertical drift
- Detects staleness (>0.5s no new data) and stops publishing
- Wheel odom always available on `/wheel_odom` (~80Hz) for debugging

## Data Relay (bag mode)

`data_relay.py` handles GO2-W rosbag quirks:
- **90° odom rotation**: GO2 forward = ROS Y-axis, rotated to X-forward
- **180° scan rotation**: Ouster mounted backward, angles shifted by π
- **Self-referencing TF filter**: drops `base_link → base_link` transforms from bag
- **PointCloud2 relay**: deep-copies with updated timestamps
- Publishes `odom → base_link` TF directly (no RF2O needed — already in bag)

## Diffbot Physics (model.sdf)

- Wheel friction: mu=100, mu2=100 (prevents lateral sliding)
- Drive wheels at x=0.05 (slightly ahead of CoM at x=0.005) to prevent forward tipping
- Caster friction: mu=0.001 (slides freely)
- Chassis: 0.50×0.30×0.10m, 5kg

## Nav2 Configurations

- `nav2_diffbot.yaml` — differential drive: Spin recovery enabled, rotate-to-heading goal checker
- `nav2_slam.yaml` — Ackermann: forward-only, no Spin, relaxed yaw tolerance (6.28 rad)
- Common: NavfnPlanner A*, RegulatedPurePursuit 0.25 m/s, inflation radius 0.25m

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
4. **Bag replay `Message queue starved`**: rosbag player warns about queue starvation when disk I/O is slow (especially `/mnt/c/` on WSL2). Not an issue on native Linux.
5. **`data_relay.py` must be executable**: symlink-install requires `chmod +x` on Python scripts. If `run_bag.sh` fails with "executable not found", run `chmod +x src/minidog_sim/minidog_sim/data_relay.py` and rebuild.
6. **Cleanup kills launch process**: The cleanup step in bringup.launch.py `pkill -9` can sometimes kill itself. This is expected — the `OnProcessExit` handler then starts the actual nodes.

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
