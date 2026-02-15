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

**Problem**: After several consecutive aborted goals, the robot would stop making progress. Costmaps would have stale obstacle data and the planner couldn't find any viable path. The Ackermann robot cannot Spin in place, so the default Nav2 recovery behaviors were ineffective. A multi-stage approach (clear at 3 aborts, unstick at 6) was too slow — the robot would waste 2+ minutes before reversing when stuck in a corner.

**Solution**: Immediate unstick on **any** abort. Nav2's BT already tried its own recovery (ClearCostmap → Wait → BackUp 1.0m). If the goal still aborted, the robot is truly stuck — reverse immediately at 0.20 m/s for 5s, then clear costmaps and pick a new goal. A custom behavior tree (`nav2_bt_ackermann.xml`) removes Spin recovery and uses only ClearCostmap + Wait + BackUp.

## 7. Infinite costmap clearing loop (resolved by simplification)

**Problem**: The multi-stage abort escalation (`_consecutive_aborts == 3` → clear, `== 6` → unstick) was brittle. An exact-match condition (`== 3`) kept re-triggering because the counter wasn't advanced past the threshold, creating an infinite clearing loop.

**Solution**: Eliminated multi-stage escalation entirely. Now any abort triggers immediate unstick (reverse + clear). The abort counter resets to 0 when unstick starts, so there's no threshold logic to get stuck on. Simpler code, faster recovery.

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

## 12. Robust odometry failure handling (Clean Architecture)

**Problem**: When scan matching failed (e.g., due to featureless scans or Gazebo glitches), `rf2o_laser_odometry` stopped publishing or reused old timestamps. This caused the TF tree to become stale, leading to `planner_server` crashes (SIGABRT due to missing transforms).

**Solution**: Implemented an external `odom_stabilizer` node. It subscribes to RF2O's raw output (with RF2O's internal TF disabled) and maintains a high-frequency `odom` TF broadcaster. If RF2O fails to update, the stabilizer republishes the last known pose with a current timestamp, keeping the TF tree alive and preventing downstream crashes.

## 13. External safety filtering for third-party nodes

**Problem**: RF2O crashed on near-zero time increments between scans. Modifying the 3rd-party source code is a maintenance burden.

**Solution**: Implemented an external `scan_safety_filter` node. It pre-processes the `/scan` topic, dropping any messages that would cause a division-by-zero in downstream nodes. This allows us to keep the `rf2o` source code unmodified while still gaining full system stability.

## 14. Message reference mutation in odom_stabilizer

**Problem**: The `odom_stabilizer` node created a new Odometry message, then immediately overwrote it with a reference to the stored message, then mutated the timestamp in-place. This caused race conditions and timestamp corruption when other nodes held references to the same message object.

**Solution**: Initially used `deepcopy()`, but this was unnecessarily heavy for a simple Odometry message. Replaced with targeted field copy — construct a new `Odometry()` and copy individual fields (header, pose, twist) explicitly. Also added staleness detection (`max_stale_sec: 0.5`) to stop republishing when the upstream RF2O source stops updating, preventing stale TF data from masking failures.

## 15. Double-increment of consecutive aborts counter

**Problem**: When a navigation goal timed out due to wall-clock timeout AND later returned an ABORTED status, the `_consecutive_aborts` counter was incremented twice: once in `_tick()` during timeout handling and again in `_on_result()` callback. This triggered unstick mechanism prematurely.

**Solution**: Added a guard at the beginning of `_on_result()` to check if `goal_in_flight` is False (indicating timeout already handled the goal), and return early to prevent double-counting.

## 16. Launch dependency race conditions

**Problem**: `scan_filter`, `rf2o_odom`, and `odom_stabilizer` all launched simultaneously, but had dependencies: RF2O depends on scan_filter publishing `/scan_safe`, and odom_stabilizer depends on RF2O publishing `/odom_rf2o`. This caused transient connection failures and dropped frames at startup.

**Solution**: Staggered Phase 2 launches using `TimerAction`:
- Phase 2a (0.5s): scan_filter → rf2o (dependency chain)
- Phase 2b (1.0s): odom_stabilizer (waits for RF2O)
- Phase 2c (1.5s): RViz + mux

## 17. TF lookup failure defaulting to map origin

**Problem**: When TF lookup failed, the frontier explorer defaulted robot position to (0, 0), causing it to select goals closest to map origin instead of the robot's actual position, wasting navigation time.

**Solution**: Changed logic to skip goal selection entirely when TF lookup fails, logging a warning and waiting for valid TF data on the next tick.

## 18. Premature exploration completion on blocked frontiers

**Problem**: When all frontier candidates were filtered out (blocked/unsafe), the code incorrectly incremented `_no_frontier_streak`, potentially declaring EXPLORATION COMPLETE prematurely even when unexplored areas existed.

**Solution**: Set `_all_blocked = True` when TF lookup fails or all candidates are filtered, preventing the completion streak from incrementing. Only increment when truly no frontier cells exist in the map.

## 19. Unstick mechanism cannot be preempted

**Problem**: Once the unstick mechanism activated (reverse for 4 seconds), nothing could stop it, even if autonomy was disabled, a new goal succeeded, or the robot unstuck itself.

**Solution**: Added preemption checks at the start of unstick logic to detect if autonomy is disabled or goal is no longer in flight, stopping the unstick maneuver early if conditions change.

## 20. Async costmap clearing with no wait

**Problem**: Costmap clearing service calls are async and return immediately. The explorer would send new goals 2 seconds later, potentially before costmaps actually repopulated, causing the same planning failures.

**Solution**: Added `_costmaps_clearing` flag and 2-second cooldown period. After calling clear services, the explorer waits for the cooldown before attempting new goals, allowing costmaps to repopulate.

## 21. Nav2 parameter tuning for Ackermann robot

**Problem**: Several Nav2 parameters were suboptimal for Ackermann steering:
- Controller frequency of 5Hz caused jerky steering (200ms between updates)
- Lookahead distance of 0.8m (2.67x robot length) caused wide turns in doorways
- Collision detection timeout of 1.0s allowed robot to travel full body length before detecting collision
- Progress checker timeout of 30s allowed robot to hang for too long when stuck

**Solution**: Tuned parameters for better Ackermann performance:
- Increased `controller_frequency` from 5Hz to 10Hz for smoother steering
- Reduced `lookahead_dist` from 0.8m to 0.5m for tighter path tracking
- Reduced `max_allowed_time_to_collision_up_to_carrot` from 1.0s to 0.25s for earlier collision detection
- Reduced `movement_time_allowance` from 30s to 15s for faster failure detection

## 22. Improved cluster cell sampling

**Problem**: Frontier cluster sampling took only the first 80 cells from BFS traversal order, potentially missing optimal goal cells in large clusters that were far from the traversal starting point.

**Solution**: Implemented stride-based sampling that distributes samples evenly across the entire cluster using `stride = len(cluster.cells) // max_samples`, ensuring spatial distribution of sampled cells.

## 23. Parameterizing hardcoded values

**Problem**: Several critical values were hardcoded, making tuning and experimentation difficult without code changes: unstick reverse speed (0.15 m/s), unstick duration (4.0s), min travel time (3.0s), cluster sample size (80), occupied threshold (50).

**Solution**: Converted hardcoded values to ROS parameters with sensible defaults, allowing runtime configuration via parameter files or command-line arguments.

## 24. Lookahead distance too short causing circular motion

**Problem**: Reduced `lookahead_dist` from 0.8m to 0.5m and `max_allowed_time_to_collision` from 1.0s to 0.25s caused Ackermann robot to run in circles instead of following paths. The short lookahead made the robot continuously overshoot the carrot point and chase its own tail.

**Solution**: Adjusted lookahead parameters to balanced values: `lookahead_dist: 0.6m` (was 0.5m), `min_lookahead_dist: 0.4m` (was 0.3m), `max_lookahead_dist: 1.0m` (was 0.8m), and `max_allowed_time_to_collision: 0.5s` (was 0.25s). These values provide better path tracking without the overshoot that causes circular behavior.

## 25. Yaw goal tolerance for Ackermann robots

**Problem**: `yaw_goal_tolerance: 0.50` (~29°) forced the Ackermann robot to oscillate at the goal position, endlessly trying to match the goal heading by driving forward arcs. The robot would reach the goal xy position, overshoot while turning, return, overshoot again — until the 30s timeout canceled the goal. Nearly every goal "failed" even though the robot was at the correct position.

**Solution**: Set `yaw_goal_tolerance: 6.28` (full circle). An Ackermann robot physically cannot rotate in place, so requiring any specific heading at the goal is counterproductive. Accept any heading when the robot reaches the goal position. This single change improved the success rate from ~0% to ~80%.

## 26. Heading-aware goal selection for forward-only navigation

**Problem**: Random goal selection sent the robot to goals behind it, requiring wide U-turns or 3-point maneuvers that an Ackermann robot handles poorly. The planner would generate paths starting with a sharp turn, causing oscillation at the path start.

**Solution**: Added heading-weighted scoring: `score = (distance / sqrt(cluster_size)) * (1 + heading_weight * |angle_diff| / pi)`. With `heading_weight: 2.0`, goals directly ahead cost 1x while goals behind cost 3x. Combined with setting goal orientation toward the goal direction (`atan2(gy-ry, gx-rx)`), this produces naturally smooth forward-moving paths.

## 27. Forward-only navigation with reverse-only recovery

**Problem**: Allowing `allow_reversing: true` in RegulatedPurePursuitController caused the planner to generate paths with reverse segments. For the Ackermann robot, reversing during path-following was unreliable — the robot would oscillate between forward and reverse, unable to smoothly track a path with direction changes.

**Solution**: Set `allow_reversing: false` for normal navigation. Reversing only happens in two recovery contexts: (1) Nav2 BT's `BackUp` behavior (1.0m at 0.15 m/s), and (2) the explorer's unstick mechanism (direct `/cmd_vel_nav` publish, 0.20 m/s for 5s). Both bypass the RPP controller entirely.

## 28. ROS sim-time vs wall-time consistency

**Problem**: The explorer used `time.time()` (wall clock) for blocked goal expiry and unstick timing, but `self.get_clock().now()` (ROS sim-time) for other operations. When simulation ran slower or faster than real-time, blocked goals expired at wrong rates and unstick durations were inconsistent.

**Solution**: Replaced all `time.time()` with `self.get_clock().now().nanoseconds`. Blocked goal TTLs, unstick end times, cooldowns, and goal timestamps all use ROS clock nanoseconds consistently. Removed `import time` entirely.

## 29. Single-pass frontier computation per tick

**Problem**: The `_tick()` method called `_find_frontier_clusters()` (which calls `list(self.map.data)`) and then `_pick_goal()` called it again. Each call involved a full O(width × height) scan of the occupancy grid and BFS clustering. This doubled the CPU cost per tick.

**Solution**: Compute `tick_data = list(self.map.data)` and `tick_clusters = _find_frontier_clusters(tick_data)` once at the top of `_tick()`, then pass both as arguments to `_publish_status()` and `_pick_goal()`. Single allocation, single scan, shared across all functions.
