### Navigation & exploration logic (current implementation)

The **target behavior** is: *when autonomy is enabled, the robot maps the entire area until there is essentially no unknown space left, then announces completion*.

#### Autonomy gating (state machine)

The only mode switch is `/autonomy_enabled`:
- **OFF (`false`)**: manual commands drive the robot (`/cmd_vel_manual → /cmd_vel`)
- **ON (`true`)**: Nav2 drives the robot (`/cmd_vel_nav → /cmd_vel`) and the explorer sends goals

Manual override is available even during autonomy: non-zero `/cmd_vel_manual` temporarily overrides Nav2 commands for safety or bootstrap mapping.

Autonomous exploration requires:
- `slam_toolbox` (publishes `/map` and `map→odom`)
- An odom source (default: `rf2o` laser odometry, publishes `odom→base_link`)
- Nav2 (planning/control producing `/cmd_vel_nav`)
- `minidog_frontier_explorer` (sending goals)

#### Pseudocode (starting from "autonomy enabled")

```text
on /autonomy_enabled == true:
  # 1) Command mux switches source
  minidog_cmd_mux forwards /cmd_vel_nav -> /cmd_vel
  (manual override can still temporarily win if manual Twist is non-zero)

  # 2) Explorer begins periodic ticks (every 2s)
  loop every ~2s:
    # Compute map data and clusters ONCE per tick (shared across functions)
    tick_data = list(map.data)
    tick_clusters = find_frontier_clusters(tick_data)
    publish_status(tick_clusters)

    if not enabled or done_announced:
      continue

    # --- Unstick: reverse when stuck against a wall ---
    if unstick_active:
      if autonomy disabled or goal completed:
        stop; unstick_active = false   # preempt
      elif time_remaining > 0:
        publish reverse cmd_vel (-0.20 m/s)
      else:
        stop; unstick_active = false
        clear_costmaps(); start cooldown
      continue

    # --- Costmap clearing cooldown (2s) ---
    if costmaps_clearing and cooldown_not_elapsed:
      continue

    # --- Goal timeout (30s wall-time) ---
    if goal_in_flight and elapsed > 30s:
      cancel_goal(); blacklist_region(); consecutive_aborts++
      continue

    if goal_in_flight:
      continue   # wait for Nav2 result

    if map not received:
      continue

    if cooldown not elapsed:
      continue

    if Nav2 action server not ready:
      continue

    # --- Immediate unstick on ANY abort ---
    if consecutive_aborts > 0:
      unstick_active = true
      unstick_end = now + 5s
      consecutive_aborts = 0
      continue

    # --- Goal selection (heading-aware, forward-biased) ---
    expire_old_blocked_goals()
    robot_pose = get_robot_pose()  # (x, y, yaw) from TF
    if robot_pose is None:
      skip this tick (TF failure)

    candidates = []
    for cluster in tick_clusters:
      for cell in stride_sample(cluster.cells):
        if blocked or too_close or not safe:
          continue
        goal_angle = atan2(wy - ry, wx - rx)
        angle_diff = |normalize(goal_angle - robot_yaw)|
        heading_penalty = 1.0 + heading_weight * angle_diff / pi
        score = (distance / sqrt(cluster_size)) * heading_penalty
        candidates.append((score, wx, wy))

    if no candidates and frontiers exist (all blocked):
      all_blocked_cycles++
      if all_blocked_cycles >= 3:
        declare EXPLORATION COMPLETE
      else:
        clear blocked list; clear costmaps; retry
      continue

    if no frontier cells at all:
      no_frontier_streak++
      if no_frontier_streak >= 8:
        declare EXPLORATION COMPLETE
      continue

    # Pick lowest-scoring (best) candidate
    goal = candidates.sort_by_score()[0]
    goal_yaw = atan2(gy - ry, gx - rx)
    goal.orientation = quaternion_from_yaw(goal_yaw)  # tangential approach

    send NavigateToPose(goal, frame="map")

    wait for result:
      if SUCCEEDED and elapsed >= 1s:
        goals_succeeded++; consecutive_aborts = 0
      if SUCCEEDED and elapsed < 1s:
        blacklist (goal reached too fast = likely invalid)
      if ABORTED:
        consecutive_aborts++; blacklist
      if CANCELED:
        log and continue
```

#### Goal selection logic (heading-aware frontier policy)

The explorer implements a **heading-weighted frontier heuristic**:

1. **Frontier detection**: Find cells where `value == 0` (free) AND any 4-neighbor is `-1` (unknown). Includes map edge cells.

2. **Clustering**: 8-connected BFS groups contiguous frontier cells. Clusters with fewer than `min_cluster_size` (5) cells are discarded.

3. **Sampling**: Large clusters are stride-sampled (`stride = cluster_size // 80`) for efficiency while maintaining spatial distribution.

4. **Safety check**: Each candidate cell must have no occupied cell (value > 50) within a 5-cell (0.25m) radius.

5. **Scoring**: `score = (distance / sqrt(cluster_size)) * heading_penalty`
   - `distance`: Euclidean from robot to candidate
   - `sqrt(cluster_size)`: larger frontiers (more information gain) get lower scores
   - `heading_penalty = 1.0 + heading_weight * |angle_diff| / pi`
     - `heading_weight` default: 2.0
     - Goal directly ahead (0°): penalty = 1.0
     - Goal to the side (90°): penalty = 2.0
     - Goal directly behind (180°): penalty = 3.0

6. **Goal orientation**: Set to the direction vector from robot to goal (`atan2(gy-ry, gx-rx)`). This produces smooth tangential paths for the Ackermann robot.

7. **Blocked goals**: Aborted/timed-out goals are blacklisted (1.5m radius, 90s TTL using ROS sim-time).

#### Navigation behavior (Ackermann-optimized)

- **Forward-only**: `allow_reversing: false` in RegulatedPurePursuitController. The robot only moves forward during normal path following.
- **Yaw tolerance**: `yaw_goal_tolerance: 6.28` (full circle). Ackermann robots cannot rotate in place to match a heading — accept any heading at goal position.
- **Recovery behavior tree** (`nav2_bt_ackermann.xml`): No Spin recovery. Uses ClearCostmap → Wait (3s) → BackUp (1.0m at 0.15 m/s). 3 retries before reporting failure.
- **Reversing for recovery only**: The `BackUp` behavior in the BT and the explorer's unstick mechanism both reverse — but only as last-resort recovery, not during normal path following.

#### Stuck detection and recovery

The system uses a **two-layer recovery** approach:

1. **Nav2 BT layer**: On controller/planner failure within a single goal, the behavior tree cycles through ClearCostmap → Wait → BackUp. Up to 3 retries before aborting the goal.

2. **Explorer layer**: On **any** goal abort (after Nav2's BT exhausted its own recovery):
   - Immediately activates unstick: reverse at 0.20 m/s for 5s
   - After unstick completes: stop, clear both costmaps, wait 2s cooldown
   - Then picks a new goal (heading-biased away from the stuck area)

This means stuck recovery is fast — no multi-abort threshold. If the robot can't reach a goal after Nav2's own retries, it backs up immediately.

#### Completion detection

Two paths to `EXPLORATION COMPLETE`:

1. **No frontiers**: 8 consecutive ticks (~16s) with zero frontier cells in the map. The stability window prevents false positives from brief SLAM map update gaps.

2. **All blocked**: If frontiers exist but all candidates are blocked/unsafe, the explorer clears the blocklist and retries. After 3 such cycles with no progress, it declares completion (all remaining frontiers are unreachable).

The explorer also distinguishes TF lookup failures from true "no frontiers" — a failed TF lookup does not increment the completion streak.

#### Key parameters

| Parameter | Default | Description |
|---|---|---|
| `heading_weight` | 2.0 | Forward bias strength (0=none, 2=3x penalty for rear goals) |
| `done_threshold_ticks` | 8 | Consecutive no-frontier ticks before declaring done |
| `goal_cooldown_sec` | 2.0 | Minimum time between goal attempts |
| `goal_wall_timeout_sec` | 30.0 | Cancel goal if no result after this long |
| `blocked_goal_radius_m` | 1.5 | Blacklist radius around failed goals |
| `blocked_goal_ttl_sec` | 90.0 | Time before blocked goals expire |
| `goal_safety_margin_cells` | 5 | Clearance from occupied cells (0.25m) |
| `min_cluster_size` | 5 | Minimum frontier cluster size |
| `min_goal_distance_m` | 0.8 | Ignore goals closer than this |
| `unstick_reverse_speed_mps` | 0.20 | Reverse speed during unstick |
| `unstick_duration_sec` | 5.0 | Duration of reverse unstick maneuver |
| `min_travel_time_sec` | 1.0 | Goals completed faster are blacklisted |
| `cluster_sample_size` | 80 | Max cells sampled per cluster |
| `occupied_threshold` | 50 | OccupancyGrid value considered occupied |
