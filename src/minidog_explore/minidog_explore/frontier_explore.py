"""Frontier-based exploration node for the minidog robot.

Finds frontier cells (free cells adjacent to unknown cells) in the
occupancy grid, clusters them via BFS, and sends Nav2 goals to
the nearest cluster centroid. Announces when all accessible areas
have been mapped.
"""

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty


@dataclass
class MapMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


@dataclass
class FrontierCluster:
    """A group of contiguous frontier cells."""
    cells: List[Tuple[int, int]] = field(default_factory=list)
    centroid_x: float = 0.0
    centroid_y: float = 0.0


def idx(x: int, y: int, w: int) -> int:
    return y * w + x


class FrontierExplorer(Node):
    """
    Frontier exploration node:
    - Subscribes /map (OccupancyGrid)
    - Finds frontier cells: free (0) adjacent to unknown (-1)
    - Clusters contiguous frontier cells via BFS
    - Picks the nearest cluster centroid to the robot (via TF)
    - Sends Nav2 NavigateToPose goals
    - Announces "DONE" when all accessible areas are mapped
    """

    def __init__(self):
        super().__init__("minidog_frontier_explorer")

        # Parameters
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("enable_topic", "/autonomy_enabled")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "minidog/base_footprint")
        self.declare_parameter("min_frontier_cells", 3)
        self.declare_parameter("goal_cooldown_sec", 2.0)
        self.declare_parameter("goal_margin_cells", 3)
        self.declare_parameter("blocked_goal_radius_m", 1.5)
        self.declare_parameter("blocked_goal_ttl_sec", 90.0)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("done_threshold_ticks", 15)
        # Minimum cluster size (cells) to be considered a valid exploration target
        self.declare_parameter("min_cluster_size", 5)
        # Minimum distance from robot to goal (avoids useless close goals at startup)
        self.declare_parameter("min_goal_distance_m", 0.8)
        # Safety margin: goal cell must have no occupied cell within N cells
        self.declare_parameter("goal_safety_margin_cells", 5)
        # Unstick mechanism parameters
        self.declare_parameter("unstick_reverse_speed_mps", 0.15)
        self.declare_parameter("unstick_duration_sec", 4.0)
        # Minimum travel time before considering a goal "succeeded too fast"
        self.declare_parameter("min_travel_time_sec", 3.0)
        # Maximum cells to sample per frontier cluster
        self.declare_parameter("cluster_sample_size", 80)
        # Occupancy threshold for considering a cell as occupied
        self.declare_parameter("occupied_threshold", 50)
        # Wall-time timeout for stuck goals
        self.declare_parameter("goal_wall_timeout_sec", 30.0)

        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        enable_topic = self.get_parameter("enable_topic").get_parameter_value().string_value
        self.goal_frame = self.get_parameter("goal_frame").get_parameter_value().string_value
        self.robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
        self.min_frontier_cells = (
            self.get_parameter("min_frontier_cells").get_parameter_value().integer_value
        )
        self.goal_cooldown_sec = (
            self.get_parameter("goal_cooldown_sec").get_parameter_value().double_value
        )
        self.goal_margin_cells = (
            self.get_parameter("goal_margin_cells").get_parameter_value().integer_value
        )
        self.blocked_goal_radius_m = (
            self.get_parameter("blocked_goal_radius_m").get_parameter_value().double_value
        )
        self.blocked_goal_ttl_sec = (
            self.get_parameter("blocked_goal_ttl_sec").get_parameter_value().double_value
        )
        self.min_cluster_size = (
            self.get_parameter("min_cluster_size").get_parameter_value().integer_value
        )
        self.min_goal_distance_m = (
            self.get_parameter("min_goal_distance_m").get_parameter_value().double_value
        )
        self._done_threshold = (
            self.get_parameter("done_threshold_ticks").get_parameter_value().integer_value
        )
        self.goal_safety_margin_cells = (
            self.get_parameter("goal_safety_margin_cells").get_parameter_value().integer_value
        )
        self.unstick_reverse_speed = (
            self.get_parameter("unstick_reverse_speed_mps").get_parameter_value().double_value
        )
        self.unstick_duration = (
            self.get_parameter("unstick_duration_sec").get_parameter_value().double_value
        )
        self._min_travel_sec = (
            self.get_parameter("min_travel_time_sec").get_parameter_value().double_value
        )
        self.cluster_sample_size = (
            self.get_parameter("cluster_sample_size").get_parameter_value().integer_value
        )
        self.occupied_threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        )
        self._goal_wall_timeout_sec = (
            self.get_parameter("goal_wall_timeout_sec").get_parameter_value().double_value
        )

        # State
        self.enabled = self.get_parameter("start_enabled").get_parameter_value().bool_value
        self.last_goal_time = None
        self.map: Optional[OccupancyGrid] = None
        self.meta: Optional[MapMeta] = None

        # Action client for Nav2
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_in_flight = False
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self._goal_sent_wall_time: float = 0.0
        self.blocked_goals: List[Tuple[float, float, float]] = []

        # TF2 for robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Exploration completion
        self._no_frontier_streak: int = 0
        self._done_announced: bool = False
        self._all_blocked: bool = False
        self._goals_sent_total: int = 0
        self._goals_succeeded: int = 0
        self._current_goal_handle = None

        # Abort streak tracking for costmap clearing and unstick
        self._consecutive_aborts: int = 0
        self._max_aborts_before_clear: int = 3
        self._max_aborts_before_unstick: int = 6
        self._unstick_active: bool = False
        self._unstick_end_time: float = 0.0

        # Costmap clearing cooldown
        self._costmaps_clearing: bool = False
        self._clear_start_time: float = 0.0
        self._clear_cooldown_sec: float = 2.0

        # Publisher for manual backup commands when stuck
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Service clients for costmap clearing
        self._clear_global_cli = self.create_client(
            Empty, '/global_costmap/clear_entirely_global_costmap'
        )
        self._clear_local_cli = self.create_client(
            Empty, '/local_costmap/clear_entirely_local_costmap'
        )

        # Subscribers
        self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 10)

        # Main tick - use 2s period to give executor time between ticks
        self.create_timer(2.0, self._tick)

        self.get_logger().info(
            f"frontier_explorer: map={map_topic} enable={enable_topic} "
            f"frame={self.goal_frame} robot_frame={self.robot_frame}"
        )
        if self.enabled:
            self.get_logger().info("Autonomy AUTO-ENABLED — will explore when Nav2 is ready")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_enable(self, msg: Bool):
        prev = self.enabled
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.goal_in_flight = False
        if self.enabled and not prev:
            self.get_logger().info("Autonomy ENABLED — starting exploration")
            # Reset completion state when re-enabled
            self._no_frontier_streak = 0
            self._done_announced = False

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.meta = MapMeta(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    def _neighbors4(self, x: int, y: int) -> List[Tuple[int, int]]:
        return [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]

    def _in_bounds(self, x: int, y: int) -> bool:
        assert self.meta is not None
        return 0 <= x < self.meta.width and 0 <= y < self.meta.height

    def _is_frontier_cell(self, x: int, y: int, data) -> bool:
        assert self.meta is not None
        v = data[idx(x, y, self.meta.width)]
        if v != 0:  # Must be free
            return False
        for nx, ny in self._neighbors4(x, y):
            if not self._in_bounds(nx, ny):
                continue
            if data[idx(nx, ny, self.meta.width)] == -1:  # Adjacent to unknown
                return True
        return False

    def _cell_to_world(self, x: int, y: int) -> Tuple[float, float]:
        assert self.meta is not None
        wx = self.meta.origin_x + (x + 0.5) * self.meta.resolution
        wy = self.meta.origin_y + (y + 0.5) * self.meta.resolution
        return wx, wy

    def _is_cell_safe(self, cx: int, cy: int, data) -> bool:
        """Check cell has no occupied neighbors within safety margin.

        Prevents sending goals right next to walls where the robot
        would get trapped by costmap inflation.
        """
        assert self.meta is not None
        w = self.meta.width
        h = self.meta.height
        margin = self.goal_safety_margin_cells
        for dy in range(-margin, margin + 1):
            for dx in range(-margin, margin + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    val = data[idx(nx, ny, w)]
                    if val > self.occupied_threshold:  # Occupied
                        return False
        return True

    # ------------------------------------------------------------------
    # Costmap clearing
    # ------------------------------------------------------------------
    def _clear_costmaps(self):
        """Clear both costmaps to recover from planner traps."""
        self.get_logger().info("Clearing costmaps to recover from planner failures")
        req = Empty.Request()
        if self._clear_global_cli.service_is_ready():
            self._clear_global_cli.call_async(req)
        if self._clear_local_cli.service_is_ready():
            self._clear_local_cli.call_async(req)
        # Set cooldown to allow costmaps to repopulate
        self._costmaps_clearing = True
        self._clear_start_time = time.time()

    # ------------------------------------------------------------------
    # TF-based robot position
    # ------------------------------------------------------------------
    def _get_robot_position(self) -> Optional[Tuple[float, float]]:
        """Get robot position in the goal_frame (map) via TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    # ------------------------------------------------------------------
    # Frontier clustering
    # ------------------------------------------------------------------
    def _find_frontier_clusters(self) -> List[FrontierCluster]:
        """Find all frontier cells and cluster them via BFS."""
        if self.map is None or self.meta is None:
            return []

        data = list(self.map.data)
        w = self.meta.width
        h = self.meta.height

        # Step 1: find all frontier cells
        frontier_set = set()
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if self._is_frontier_cell(x, y, data):
                    frontier_set.add((x, y))

        if len(frontier_set) < self.min_frontier_cells:
            return []

        # Step 2: cluster via BFS (8-connected)
        visited = set()
        clusters = []
        for cell in frontier_set:
            if cell in visited:
                continue
            cluster = FrontierCluster()
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cx, cy = queue.popleft()
                cluster.cells.append((cx, cy))
                # 8-connected neighbors
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = cx + dx, cy + dy
                        if (nx, ny) in frontier_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            queue.append((nx, ny))
            if len(cluster.cells) >= self.min_cluster_size:
                # Compute centroid in world coordinates
                sum_x, sum_y = 0.0, 0.0
                for x, y in cluster.cells:
                    wx, wy = self._cell_to_world(x, y)
                    sum_x += wx
                    sum_y += wy
                cluster.centroid_x = sum_x / len(cluster.cells)
                cluster.centroid_y = sum_y / len(cluster.cells)
                clusters.append(cluster)

        return clusters

    # ------------------------------------------------------------------
    # Goal picking
    # ------------------------------------------------------------------
    def _is_blocked(self, wx: float, wy: float) -> bool:
        """Check if a world position is in a blocked region."""
        r2 = self.blocked_goal_radius_m ** 2
        for bx, by, _exp in self.blocked_goals:
            if (wx - bx) ** 2 + (wy - by) ** 2 <= r2:
                return True
        return False

    def _pick_goal(self) -> Optional[Tuple[float, float]]:
        """Pick the best exploration goal from frontier clusters.

        Returns:
            (x, y) goal in world coordinates, or None if no frontiers exist.
            Returns "BLOCKED" sentinel (stored in self._all_blocked) when
            frontiers exist but are all blocked — this is NOT "done".
        """
        # Expire old blocked goals
        now_wall = time.time()
        self.blocked_goals = [
            (x, y, exp) for (x, y, exp) in self.blocked_goals if exp > now_wall
        ]

        clusters = self._find_frontier_clusters()
        if not clusters:
            self._all_blocked = False
            return None

        # Get robot position for nearest-cluster selection
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self.get_logger().warn("Cannot get robot TF; skipping goal selection this tick")
            # Set _all_blocked to prevent incrementing done streak on TF failures
            self._all_blocked = True
            return None

        rx, ry = robot_pos
        data = list(self.map.data) if self.map else []

        # Pick the nearest SAFE frontier cell to the robot.
        # Safe = not blocked, far enough from walls, far enough from robot.
        candidates: List[Tuple[float, float, float]] = []  # (dist, wx, wy)
        for cluster in clusters:
            # Sample spatially distributed cells using stride-based sampling
            max_samples = self.cluster_sample_size
            if len(cluster.cells) > max_samples:
                stride = len(cluster.cells) // max_samples
                sample = cluster.cells[::stride]
            else:
                sample = cluster.cells
            for cell_x, cell_y in sample:
                wx, wy = self._cell_to_world(cell_x, cell_y)
                if self._is_blocked(wx, wy):
                    continue
                d = math.hypot(wx - rx, wy - ry)
                if d < self.min_goal_distance_m:
                    continue
                # Safety check: is the cell far enough from walls?
                if data and self.meta and not self._is_cell_safe(cell_x, cell_y, data):
                    continue
                candidates.append((d, wx, wy))

        if not candidates:
            # Frontiers exist but all blocked — NOT done, just waiting
            self._all_blocked = True
            return None

        self._all_blocked = False
        candidates.sort(key=lambda c: c[0])
        return (candidates[0][1], candidates[0][2])

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def _tick(self):
        if not self.enabled:
            return
        if self._done_announced:
            return
        # Unstick: when stuck against a wall, manually reverse for 3s
        if self._unstick_active:
            # Preempt unstick if autonomy disabled or goal completed
            if not self.enabled or not self.goal_in_flight:
                cmd = Twist()  # Stop
                self._cmd_vel_pub.publish(cmd)
                self._unstick_active = False
                self.get_logger().info("Unstick preempted (autonomy disabled or goal completed)")
                return

            if time.time() < self._unstick_end_time:
                cmd = Twist()
                cmd.linear.x = -self.unstick_reverse_speed  # Reverse slowly
                self._cmd_vel_pub.publish(cmd)
                return
            else:
                # Unstick done, stop and clear costmaps
                cmd = Twist()
                self._cmd_vel_pub.publish(cmd)
                self._unstick_active = False
                self._consecutive_aborts = 0
                self._clear_costmaps()
                self.get_logger().info("Unstick complete, cleared costmaps")
                return

        # Wait for costmap clearing cooldown
        if self._costmaps_clearing:
            if time.time() - self._clear_start_time < self._clear_cooldown_sec:
                return  # Wait for costmaps to repopulate
            self._costmaps_clearing = False
            self.get_logger().info("Costmap clearing cooldown complete")

        if self.goal_in_flight:
            # Cancel goals stuck for too long (wall time)
            if self._goal_sent_wall_time > 0:
                elapsed = time.time() - self._goal_sent_wall_time
                if elapsed > self._goal_wall_timeout_sec:
                    self.get_logger().warn(
                        f"Goal stuck for {elapsed:.0f}s wall time; canceling"
                    )
                    if self._current_goal_handle is not None:
                        try:
                            self._current_goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                    self.goal_in_flight = False
                    self._consecutive_aborts += 1
                    if self.last_goal_xy:
                        bx, by = self.last_goal_xy
                        self.blocked_goals.append(
                            (bx, by, time.time() + self.blocked_goal_ttl_sec)
                        )
            return
        if self.map is None:
            return

        now = self.get_clock().now()
        if self.last_goal_time is not None:
            age = (now - self.last_goal_time).nanoseconds / 1e9
            if age < self.goal_cooldown_sec:
                return

        if not self.nav.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready (navigate_to_pose)")
            return

        # After many consecutive aborts, try to unstick by reversing
        if self._consecutive_aborts >= self._max_aborts_before_unstick:
            self.get_logger().info(
                f"Stuck: {self._consecutive_aborts} aborts, reversing to unstick"
            )
            self._unstick_active = True
            self._unstick_end_time = time.time() + self.unstick_duration
            self._consecutive_aborts = 0
            return
        # Clear costmaps at exactly N aborts (use exact match, not modulo)
        if self._consecutive_aborts == self._max_aborts_before_clear:
            self._clear_costmaps()
            # Don't reset counter, don't return - proceed to pick a goal

        goal_xy = self._pick_goal()
        if goal_xy is None:
            if hasattr(self, '_all_blocked') and self._all_blocked:
                if self.blocked_goals:
                    self.get_logger().info(
                        f"All {len(self.blocked_goals)} blocked goals -- clearing & retrying"
                    )
                    self.blocked_goals.clear()
                    self._clear_costmaps()
                return
            # Truly no frontier cells in the map
            self._no_frontier_streak += 1
            if self._no_frontier_streak >= self._done_threshold:
                self.get_logger().info(
                    "================================================"
                )
                self.get_logger().info(
                    "  EXPLORATION COMPLETE — all accessible areas"
                )
                self.get_logger().info(
                    f"  have been mapped! ({self._goals_sent_total} goals sent, "
                    f"{self._goals_succeeded} succeeded)"
                )
                self.get_logger().info(
                    "  DONE!"
                )
                self.get_logger().info(
                    "================================================"
                )
                self._done_announced = True
            else:
                self.get_logger().info(
                    f"No frontiers ({self._no_frontier_streak}/{self._done_threshold} toward done)"
                )
            return

        # Reset streak when frontiers found
        self._no_frontier_streak = 0

        gx, gy = goal_xy
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = now.to_msg()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        robot_pos = self._get_robot_position()
        dist_str = ""
        if robot_pos:
            d = math.hypot(gx - robot_pos[0], gy - robot_pos[1])
            dist_str = f" dist={d:.1f}m"

        self.get_logger().info(
            f"Goal #{self._goals_sent_total + 1}: x={gx:.2f} y={gy:.2f}{dist_str}"
        )

        self.goal_in_flight = True
        self.last_goal_time = now
        self.last_goal_xy = (gx, gy)
        self._goals_sent_total += 1

        send_future = self.nav.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    # ------------------------------------------------------------------
    # Nav2 action callbacks
    # ------------------------------------------------------------------
    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.goal_in_flight = False
            if self.last_goal_xy:
                bx, by = self.last_goal_xy
                self.blocked_goals.append(
                    (bx, by, time.time() + self.blocked_goal_ttl_sec)
                )
            return
        self._current_goal_handle = goal_handle
        self._goal_sent_wall_time = time.time()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            # Guard against double-processing if timeout already handled this goal
            if not self.goal_in_flight:
                return

            result = future.result()
            status = result.status
            elapsed = (
                time.time() - self._goal_sent_wall_time
                if self._goal_sent_wall_time > 0
                else 999.0
            )
            if status == GoalStatus.STATUS_SUCCEEDED:
                if elapsed < self._min_travel_sec:
                    self.get_logger().warn(
                        f"SUCCEEDED too fast ({elapsed:.1f}s); blacklisting"
                    )
                    if self.last_goal_xy:
                        bx, by = self.last_goal_xy
                        self.blocked_goals.append(
                            (bx, by, time.time() + self.blocked_goal_ttl_sec)
                        )
                else:
                    self._goals_succeeded += 1
                    self._consecutive_aborts = 0
                    self.get_logger().info(
                        f"Goal SUCCEEDED ({elapsed:.1f}s)"
                    )
            elif status == GoalStatus.STATUS_ABORTED:
                self._consecutive_aborts += 1
                self.get_logger().warn(
                    f"Goal ABORTED ({elapsed:.1f}s) "
                    f"[streak: {self._consecutive_aborts}]"
                )
                if self.last_goal_xy:
                    bx, by = self.last_goal_xy
                    self.blocked_goals.append(
                        (bx, by, time.time() + self.blocked_goal_ttl_sec)
                    )
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Goal CANCELED")
            else:
                self.get_logger().info(f"Goal finished status={status}")
        except Exception as e:
            self.get_logger().error(f"Error getting result: {e}")
        finally:
            self.goal_in_flight = False


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
