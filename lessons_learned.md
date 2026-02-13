# Lessons learned

Problems encountered and resolved while developing the autonomous exploration system.

## 1. rf2o scan odometry drift and instability

**Problem**: `rf2o_laser_odometry` produced noisy TF at sim startup and would drift in featureless areas, causing SLAM to accumulate error and Nav2 to plan into walls.

**Solution**: Switched to Gazebo's wheel odometry (`odom_source:=wheel`) for simulation. rf2o remains available for real hardware where wheel odometry is unreliable. The `minidog_scan_odom` node republishes Gazebo GT odom with correct TF frames.

## 2. ActionClient.wait_for_server blocks DDS discovery

**Problem**: The frontier explorer called `self.nav.wait_for_server(timeout_sec=0.5)` inside a timer callback. This blocked the `rclpy` executor, preventing DDS discovery messages from being processed. Result: the explorer permanently logged "Nav2 action server not ready" even though `ros2 action list` showed the server was running.

**Solution**: Replaced with non-blocking `self.nav.server_is_ready()` check and increased the timer period from 1s to 2s to give the executor more breathing room between ticks.

## 3. SLAM overwhelmed by 10Hz lidar scans

**Problem**: `slam_toolbox` could not process 10Hz lidar input in real time on WSL2. Log was flooded with "Message Filter dropping message: frame 'minidog/base_footprint/ouster' for reason 'discarding message because the queue is full'". Simulation ran at ~1/16th real-time.

**Solution**: Added `throttle_scans: 5` to `slam_toolbox_async.yaml`, reducing effective processing rate to 2Hz. Also reduced costmap update frequencies (global: 1Hz, local: 2Hz) and set Gazebo `real_time_factor` to 1.0.

## 4. Costmap inflation radius causing planner deadlock

**Problem**: `inflation_radius: 0.30` was larger than necessary for the 0.225m inscribed radius. In tight spaces, the planner saw the robot's current position as being in lethal cost, making all goals unreachable ("GridBased: failed to create plan").

**Solution**: Reduced `inflation_radius` to 0.25 (just above inscribed radius) and `cost_scaling_factor` from 5.0 to 3.0. This kept safety margins while allowing the planner to find paths through doorways.

## 5. Goals placed too close to walls

**Problem**: Frontier cells by definition are adjacent to unknown space, which is often next to walls. The explorer would send goals right at the wall edge, where Nav2 would abort due to costmap obstacles.

**Solution**: Added a `goal_safety_margin` (5 cells / 0.25m). Before accepting a frontier cell as a goal candidate, the explorer checks that no occupied cell exists within this radius. This pushes goals into safe free space.

## 6. Robot getting stuck with no recovery

**Problem**: After several consecutive aborted goals, the robot would stop making progress. Costmaps would have stale obstacle data and the planner couldn't find any viable path. The Ackermann robot cannot Spin in place, so the default Nav2 recovery behaviors were ineffective.

**Solution**: Implemented a multi-stage recovery system:
- After 3 consecutive aborts: clear both global and local costmaps
- After 6 consecutive aborts: publish reverse `Twist` command for 4 seconds (manual unstick), then clear costmaps
- A custom behavior tree (`nav2_bt_ackermann.xml`) removes Spin recovery and uses only ClearCostmap + Wait + BackUp

## 7. Infinite costmap clearing loop

**Problem**: The `_tick()` method checked `if _consecutive_aborts == _max_aborts_before_clear` (3) and called `_clear_costmaps()`. But clearing didn't change the abort counter, so the condition remained true on every subsequent tick, creating an infinite clearing loop.

**Solution**: Restructured the logic so costmap clearing triggers once at the threshold, then the counter continues accumulating toward the unstick threshold (6). Only a successful goal or a completed unstick sequence resets the counter.

## 8. Zombie processes from previous runs

**Problem**: Gazebo, Nav2, SLAM, and bridge processes survived after Ctrl+C or crashes. New launches would fail due to port conflicts, stale DDS participants, or conflicting TF publishers.

**Solution**: `run.sh` includes an aggressive `kill_zombies()` function that `pkill -9 -f` matches 20+ process patterns. It runs twice with a 1s gap (second pass catches reparented children). Also cleans DDS shared-memory files (`/dev/shm/fastrtps_*`).

## 9. Race conditions at startup

**Problem**: If SLAM, Nav2, and the explorer all start simultaneously, they fail because dependencies are not yet available (no TF tree, no `/map`, no action server).

**Solution**: Staggered `TimerAction` delays in `bringup.launch.py`: Gazebo at 0s, SLAM at 5s, Nav2 at 10s, lifecycle_manager at 15s, explorer at 20s.

## 10. Premature "exploration complete" announcements

**Problem**: The explorer would see zero frontiers briefly (e.g., between SLAM map updates) and declare exploration complete while rooms remained unexplored.

**Solution**: Added a stability threshold: completion requires 15 consecutive ticks (~30s) with no frontier cells. Also distinguishes "no frontiers found" from "all frontiers blocked" — when all frontiers are in the blocked list, it clears the list and retries instead of declaring completion.

## 11. DDS shared-memory issues on WSL2

**Problem**: FastDDS shared-memory transport would leave stale segments in `/dev/shm/` after crashes, causing new ROS 2 nodes to hang during initialization.

**Solution**: Configured `fastdds_no_shm.xml` to disable shared-memory transport (UDP only). The `run.sh` cleanup also removes stale `/dev/shm/fastrtps_*` and `/dev/shm/sem.fastrtps_*` files.
