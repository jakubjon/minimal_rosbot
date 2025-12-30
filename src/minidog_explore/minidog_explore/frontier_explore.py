import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool


@dataclass
class MapMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


def idx(x: int, y: int, w: int) -> int:
    return y * w + x


class FrontierExplorer(Node):
    """
    Minimal frontier exploration:
    - subscribe /map (OccupancyGrid)
    - find frontier cells: free (0) adjacent to unknown (-1)
    - pick the farthest frontier cell from origin (0,0) to encourage coverage
    - send Nav2 NavigateToPose goals repeatedly while enabled

    This is intentionally simple (sandbox). It mirrors the qre_go2 idea of modular roles.
    """

    def __init__(self):
        super().__init__("minidog_frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("enable_topic", "/autonomy_enabled")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("min_frontier_cells", 30)
        self.declare_parameter("goal_cooldown_sec", 5.0)

        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        enable_topic = self.get_parameter("enable_topic").get_parameter_value().string_value
        self.goal_frame = self.get_parameter("goal_frame").get_parameter_value().string_value
        self.min_frontier_cells = (
            self.get_parameter("min_frontier_cells").get_parameter_value().integer_value
        )
        self.goal_cooldown_sec = (
            self.get_parameter("goal_cooldown_sec").get_parameter_value().double_value
        )

        self.enabled = False
        self.last_goal_time = None
        self.map: Optional[OccupancyGrid] = None
        self.meta: Optional[MapMeta] = None

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_in_flight = False

        self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 10)

        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"frontier_explorer: map={map_topic} enable={enable_topic} action=navigate_to_pose"
        )

    def _on_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.goal_in_flight = False

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.meta = MapMeta(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )

    def _neighbors4(self, x: int, y: int) -> List[Tuple[int, int]]:
        return [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]

    def _in_bounds(self, x: int, y: int) -> bool:
        assert self.meta is not None
        return 0 <= x < self.meta.width and 0 <= y < self.meta.height

    def _is_frontier_cell(self, x: int, y: int, data: List[int]) -> bool:
        assert self.meta is not None
        v = data[idx(x, y, self.meta.width)]
        if v != 0:
            return False
        for nx, ny in self._neighbors4(x, y):
            if not self._in_bounds(nx, ny):
                continue
            if data[idx(nx, ny, self.meta.width)] == -1:
                return True
        return False

    def _cell_to_world(self, x: int, y: int) -> Tuple[float, float]:
        assert self.meta is not None
        wx = self.meta.origin_x + (x + 0.5) * self.meta.resolution
        wy = self.meta.origin_y + (y + 0.5) * self.meta.resolution
        return wx, wy

    def _pick_goal(self) -> Optional[Tuple[float, float]]:
        if self.map is None or self.meta is None:
            return None

        data = list(self.map.data)
        frontier_cells: List[Tuple[int, int]] = []

        # Cheap scan for frontier cells
        for y in range(1, self.meta.height - 1):
            for x in range(1, self.meta.width - 1):
                if self._is_frontier_cell(x, y, data):
                    frontier_cells.append((x, y))

        if len(frontier_cells) < self.min_frontier_cells:
            return None

        # Choose the farthest from map origin (simple, deterministic)
        best = None
        best_d2 = -1.0
        for x, y in frontier_cells:
            wx, wy = self._cell_to_world(x, y)
            d2 = wx * wx + wy * wy
            if d2 > best_d2:
                best_d2 = d2
                best = (wx, wy)

        return best

    def _tick(self):
        if not self.enabled:
            return
        if self.goal_in_flight:
            return
        if self.map is None:
            return

        now = self.get_clock().now()
        if self.last_goal_time is not None:
            age = (now - self.last_goal_time).nanoseconds / 1e9
            if age < self.goal_cooldown_sec:
                return

        if not self.nav.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("Nav2 action server not ready (navigate_to_pose)")
            return

        goal_xy = self._pick_goal()
        if goal_xy is None:
            self.get_logger().info("No frontiers yet (or too few); keep driving to reveal map")
            return

        gx, gy = goal_xy
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = now.to_msg()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending frontier goal: x={gx:.2f} y={gy:.2f} frame={self.goal_frame}")
        self.goal_in_flight = True
        self.last_goal_time = now

        send_future = self.nav.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Frontier goal rejected")
            self.goal_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            status = future.result().status
            self.get_logger().info(f"NavigateToPose finished with status={status}")
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
        rclpy.shutdown()


