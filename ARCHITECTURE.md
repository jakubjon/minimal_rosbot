## minidog_sim architecture

This document describes how the current simulation stack is wired, how robot commanding works (manual vs autonomy), and how mapping/navigation are composed. It also summarizes the primary nodes, their states, and their interfaces (topics/TF).

### Design intent (high level)

- **Sandbox now, reusable later**: structure follows `qre_go2`’s modular launch + stable topics + strict TF ownership conventions, so the same patterns can be reused on real hardware later.
- **Default is manual**: the robot is controllable by `/cmd_vel_manual` at all times; autonomy must be explicitly enabled.
- **Preferred odom source is scan matching**: the global motion estimate should come from scan data, not wheel odometry.



### Commanding architecture (manual vs autonomy)

Manual and autonomous velocity commands are multiplexed so the robot never “accidentally” enters autonomy.

Topics:
- Manual input: `/cmd_vel_manual` (`geometry_msgs/Twist`)
- Autonomy input (Nav2 output): `/cmd_vel_nav` (`geometry_msgs/Twist`)
- Final output to robot/sim: `/cmd_vel` (`geometry_msgs/Twist`)
- Autonomy enable switch: `/autonomy_enabled` (`std_msgs/Bool`)

Mux behavior (`minidog_cmd_mux`):
- If `/autonomy_enabled == false`: forward `/cmd_vel_manual` → `/cmd_vel`
- If `/autonomy_enabled == true`: forward `/cmd_vel_nav` → `/cmd_vel`
- Manual override: if autonomy is enabled but you actively publish manual commands, the mux forwards manual (useful for safety/bootstrap).

### Simulation transport (ROS <-> Gazebo)

Gazebo is started via `ign gazebo` and the robot model is spawned from SDF.

`ros_gz_bridge/parameter_bridge` bridges:
- `/cmd_vel` (ROS) → `/model/minidog/cmd_vel` (Gazebo transport)
- `/clock`, `/joint_states`, `/wheel_odom`, `/scan`
- optionally `/tf` (only in `odom_source:=wheel`)

The robot model uses an Ackermann steering system plugin in SDF and consumes the bridged `/model/minidog/cmd_vel`.

### Mapping + navigation + exploration

#### SLAM
- `slam_toolbox` subscribes to `/scan` and `TF` and publishes:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - `map -> minidog/odom` TF

#### Scan odometry (preferred)
- `rf2o_laser_odometry_node` subscribes to `/scan` and publishes:
  - `/odom` (`nav_msgs/Odometry`) (topic)
  - `minidog/odom -> minidog/base_footprint` (TF)

#### Nav2
Nav2 is launched as individual nodes + a lifecycle manager:
- `controller_server`, `planner_server`, `bt_navigator`, `behavior_server`, `waypoint_follower`, `velocity_smoother`, `lifecycle_manager`

Command output from Nav2 is remapped so it **never publishes directly to `/cmd_vel`**:
- Nav2 publishes to `/cmd_vel_nav`, which the mux gates into `/cmd_vel`.

Behavior tree:
- `bt_navigator.default_bt_xml_filename` is set to `navigate_w_replanning_and_recovery.xml` (qre_go2 pattern) so it can **replan and try recoveries** instead of freezing permanently when blocked.

#### Frontier exploration
`minidog_frontier_explorer`:
- subscribes: `/map`, `/autonomy_enabled`
  - finds frontier cells (free cells adjacent to unknown)
  - sends `NavigateToPose` goals when autonomy is enabled
  - on ABORTED goals, it blacklists the area temporarily to avoid repeatedly hitting the same obstacle/goal


### Observability and UI

#### RViz
`rviz/robot.rviz` includes:
- `/map`, `/scan`, RobotModel, TF
- Nav2 visualization:
  - Goal marker: `/goal_pose`
  - Plans: `/plan`, `/local_plan`
  - Costmaps: `/global_costmap/costmap_raw`, `/local_costmap/costmap_raw`
  - Footprint: `/local_costmap/published_footprint`

#### Streamlit web UI (optional)
The web UI node (`/minidog_webapp`) provides:
- control: publish `/autonomy_enabled` and `/cmd_vel_manual`
- monitoring: subscribes to `/scan`, `/map`, `/tf`, `/wheel_odom`, `/cmd_vel*`, `/autonomy_enabled`

### Logging model (quiet by default)

Bringup defaults:
- `quiet_terminal:=true`
- `log_level:=warn`
- sets `RCUTILS_LOGGING_BUFFERED_STREAM=1` (qre_go2 pattern)

Most node output is routed to ROS log files under `~/.ros/log/...` for a cleaner main terminal.

### Architecture diagram (control + data flow)

```mermaid
flowchart LR
  subgraph user[UserInterfaces]
    WebUI[StreamlitWebUI]
    Rqt[RqtTeleopOrPublisher]
    Rviz[Rviz]
  end

  subgraph sim[Simulation]
    Gazebo[GazeboIgnition]
    Ackermann[AckermannSteeringPlugin]
  end

  subgraph bridge[TransportBridge]
    GzBridge[ros_gz_bridge_parameter_bridge]
  end

  subgraph core[CoreROSGraph]
    Mux[minidog_cmd_mux]
    RF2O[rf2o_laser_odometry_node]
    Slam[slam_toolbox]
    Explorer[minidog_frontier_explorer]
    Nav2[Nav2Stack]
  end

  Rqt -->|cmd_vel_manual| Mux
  WebUI -->|cmd_vel_manual| Mux
  WebUI -->|autonomy_enabled| Mux
  WebUI -->|autonomy_enabled| Explorer

  Nav2 -->|cmd_vel_nav| Mux
  Mux -->|cmd_vel| GzBridge
  GzBridge -->|model/minidog/cmd_vel| Gazebo
  Gazebo --> Ackermann

  Gazebo -->|minidog/ouster/points| GzBridge
  GzBridge -->|scan| RF2O
  GzBridge -->|scan| Slam

  RF2O -->|odom TF odom to base| Nav2
  Slam -->|map TF map to odom| Nav2
  Slam -->|map| Explorer

  Slam -->|map| Rviz
  GzBridge -->|scan| Rviz
  ```

### Node/interface summary

#### `minidog_sim` bringup components

- `ign gazebo` (process)
  - **state**: running
  - **interface**: Gazebo sim + sensors
- `ros_gz_bridge/parameter_bridge`
  - **state**: running
  - **in/out**: `/cmd_vel`, `/scan`, `/wheel_odom`, `/clock`, `/joint_states`, optionally `/tf`
- `robot_state_publisher`
  - **state**: running
  - **out**: TF for URDF links (`frame_prefix=minidog/`)
- `tf2_ros/static_transform_publisher`
  - **state**: running
  - **out**: `minidog/base_footprint -> minidog/base_footprint/ouster`
- `rviz2`
  - **state**: running (optional)
  - **in**: `/map`, `/scan`, TF, Nav2 displays

#### `rf2o_laser_odometry_node` (when `odom_source:=scan`)
- **state**: running
- **in**: `/scan`
- **out**: `/odom`, TF `minidog/odom -> minidog/base_footprint`

#### `slam_toolbox` (when enabled)
- **state**: running
- **in**: `/scan`, TF
- **out**: `/map`, TF `map -> minidog/odom`

#### `minidog_cmd_mux`
- **state**: running
- **in**: `/cmd_vel_manual`, `/cmd_vel_nav`, `/autonomy_enabled`
- **out**: `/cmd_vel`

#### Nav2 nodes (when enabled)
- **state model**: lifecycle-managed (configured/active)
- **in**: `/map`, `/scan`, `/odom`, TF
- **out**: `/cmd_vel_nav`, plus plans/costmaps/footprint topics for visualization

#### `minidog_frontier_explorer` (when enabled)
- **state**: idle unless `/autonomy_enabled==true`
- **in**: `/map`, `/autonomy_enabled`
- **out**: NavigateToPose goals (action client)

#### Streamlit web node `/minidog_webapp` (when enabled)
- **state**: running
- **in**: various monitoring topics
- **out**: `/autonomy_enabled`, `/cmd_vel_manual`


