### Navigation & exploration logic (current implementation)

The **target behavior** is: *when autonomy is enabled, the robot maps the entire area until there is essentially no unknown space left*.

Here is what the current implementation actually does today.

#### Autonomy gating (state machine)

The only mode switch is `/autonomy_enabled`:
- **OFF (`false`)**: manual commands drive the robot (`/cmd_vel_manual → /cmd_vel`)
- **ON (`true`)**: Nav2 drives the robot (`/cmd_vel_nav → /cmd_vel`) and the explorer starts sending goals

Autonomous exploration therefore requires these to be running:
- `slam_toolbox` (publishes `/map` and `map→odom`)
- an odom source (recommended: `rf2o_laser_odometry_node` publishing `odom→base`)
- Nav2 (planning/control producing `/cmd_vel_nav`)
- `minidog_frontier_explorer` (sending goals)

#### Pseudocode (starting from “autonomy enabled”)

This is a simplified pseudocode view of the *current* autonomous loop:

```text
on /autonomy_enabled == true:
  # 1) Command mux switches source
  minidog_cmd_mux forwards /cmd_vel_nav -> /cmd_vel
  (manual override can still temporarily win if manual Twist is non-zero)

  # 2) Explorer begins periodic ticks
  loop every ~1s:
    if Nav2 NavigateToPose action server not ready:
      wait; continue

    if cooldown not elapsed:
      continue

    if /map not received yet:
      continue

    frontier_cells = []
    for each cell (x,y) in OccupancyGrid:
      if cell==FREE(0) and any 4-neighbor==UNKNOWN(-1):
        frontier_cells.append((x,y))

    frontier_cells = filter_out_cells_near_map_border(frontier_cells, margin_cells)

    if frontier_cells is empty (or too few):
      # current behavior: do nothing and keep waiting
      continue

    goal_cell = argmax_over(frontier_cells, distance_to_map_origin)
    goal_xy = cell_to_world(goal_cell)
    goal_xy = clamp_inside_map(goal_xy, margin_cells)

    send NavigateToPose(goal_xy, frame="map")

    wait for result:
      if result == SUCCEEDED:
        goal_in_flight = false
        # next tick will pick a new frontier
      if result == ABORTED:
        blacklist_goal_region(goal_xy, radius, ttl)
        goal_in_flight = false
        # next tick picks a different frontier (not near blacklisted regions)

on /autonomy_enabled == false:
  minidog_cmd_mux forwards /cmd_vel_manual -> /cmd_vel
  explorer stops sending new goals
```

#### Current goal selection logic (frontier policy)

The explorer implements a simple frontier heuristic:
- Compute **frontier cells** from `/map` (`nav_msgs/OccupancyGrid`):
  - cell == **free (0)** AND has a 4-neighbor cell == **unknown (-1)**
- Filter out frontiers near the map boundary (goal margin) to avoid Nav2 `worldToMap` edge failures.
- Pick a goal as the **farthest frontier cell from the map origin** (deterministic).
- Send `NavigateToPose` in the `map` frame.

There is no clustering, no information-gain scoring, and no explicit pre-check that a plan exists before sending.

#### What happens at obstacles (why it can stop)

When the robot encounters obstacles, Nav2 may:
- replan and attempt recovery behaviors via `navigate_w_replanning_and_recovery.xml`
- still stall if the goal is unreachable or the local controller cannot find a feasible collision-free command

Common “stop” causes in this setup:
- goal is technically a frontier, but **not reachable** in the current free-space graph
- costmap inflation/footprint/controller parameters are too conservative for narrow passages
- repeated recoveries don’t resolve the blockage (e.g., boxed-in situations)

#### Current “stuck goal” handling

If `NavigateToPose` finishes with **ABORTED**, the explorer:
- blacklists the last goal region (radius + TTL)
- avoids choosing candidate frontier goals near that region temporarily

This prevents repeatedly hammering the same failed target.

#### Completion criteria (“map until no unknown exists”)

Today, there is **no explicit “done” criterion** implemented.

The explorer keeps running while autonomy is enabled and continues to send goals as long as it can find frontiers; it does not compute global unknown percentage and it does not stop automatically once the map is complete.

### Navigation & exploration logic (future improvements)

To reach the intended behavior (“explore until unknown≈0”), the typical next steps are:

#### 1) Add an explicit map-completion metric + stop condition

- Compute \(p_{unknown} = N_{unknown}/N_{total}\) where unknown cells are `-1`.
- Define exploration “done” when \(p_{unknown}\) falls below a threshold (e.g. 1–3%) for a stable window (e.g. 10–30s).
- On completion:
  - publish `/autonomy_enabled=false`
  - publish zero `/cmd_vel_manual`
  - optionally save the map (slam_toolbox service)

#### 2) Improve frontier selection quality

- Cluster frontier cells into regions (connected components).
- Score regions by **information gain** and **travel cost** (plan length).
- Choose the best frontier by maximizing a utility like `gain / cost`.
- Sample multiple candidate poses per frontier and pick one that is reachable and collision-safe.

#### 3) Feasibility pre-check before sending a goal

Before sending `NavigateToPose`, request a plan (Nav2 planner service). If planning fails, blacklist the region and pick a different frontier immediately.

#### 4) Better stuck detection

Use additional signals beyond “ABORTED”:
- progress checker failure
- oscillation conditions
- repeated recovery loops

#### 5) Systematic coverage / cleanup phase

Greedy frontier exploration often leaves pockets of unknown (occlusions, narrow passages). Add a final cleanup phase:
- target remaining unknown “islands”
- optionally run coverage path planning over free space

#### 6) Tuning for this robot (Ackermann + costmaps)

If the robot stalls too often, likely tuning targets:
- footprint/inflation radius
- controller limits (turn radius, min velocity thresholds)
- progress checker and recovery parameters (backup distance/angle, clear costmap usage)

#### Behavior Trees as a further improvement (when it makes sense)

There are two distinct “BT layers” you can use:

- **Nav2 internal BT** (already used): `bt_navigator` runs a BT for *executing a single navigation task* (go to pose, replan, recover, etc).
- **Exploration supervisor BT** (not implemented): a BT that decides *which goal to go to next* until mapping is complete.

If your goal is “explore until unknown≈0”, an exploration-supervisor BT can be a good fit because it naturally encodes:
- retries and fallbacks (try next frontier, then rotate-in-place, then backtrack)
- explicit termination conditions (unknown fraction below threshold)
- clear separation between “decision making” and Nav2’s “motion execution”

Example high-level exploration-supervisor BT (conceptual):

```text
Root
└─ ReactiveFallback
   ├─ Sequence   # completion check
   │  ├─ Condition: autonomy_enabled == true
   │  ├─ Condition: map_unknown_fraction < done_threshold (stable for T seconds)
   │  └─ Action: disable_autonomy + stop_robot + (optional) save_map
   ├─ Sequence   # main exploration loop
   │  ├─ Condition: autonomy_enabled == true
   │  ├─ Action: update_frontiers_from_map
   │  ├─ Action: select_best_frontier (gain/cost, reachable)
   │  ├─ Action: navigate_to_frontier (Nav2 NavigateToPose)
   │  └─ Action: wait_for_map_update / cooldown
   └─ Sequence   # fallback when stuck / no frontiers
      ├─ Action: clear_costmaps
      ├─ Action: recovery_motion (spin/backup)
      └─ Action: re-evaluate_frontiers
```

What this would change vs today:
- “done” becomes explicit (not just “no frontiers seen right now”)
- “try next frontier” is a first-class fallback, not only triggered by ABORTED
- you can integrate feasibility pre-checks (planner) as a dedicated action node

If you don’t want an external BT engine, you can implement the same structure as a small explicit state machine in the explorer node; BT just tends to scale better as behaviors grow.
