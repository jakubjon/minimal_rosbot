import json
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from action_msgs.msg import GoalStatusArray
import math

import streamlit as st
from streamlit_autorefresh import st_autorefresh


@dataclass
class TopicState:
    last_recv_wall_time: float = 0.0
    last_msg: Optional[Any] = None


def _now() -> float:
    return time.time()


def _age_s(ts: float) -> Optional[float]:
    if ts <= 0:
        return None
    return _now() - ts


def _fmt_age(age: Optional[float]) -> str:
    if age is None:
        return "—"
    return f"{age:0.2f}s"


def _twist_summary(msg: Twist) -> str:
    return f"lin.x={msg.linear.x:0.2f}, ang.z={msg.angular.z:0.2f}"


def _scan_summary(msg: LaserScan) -> str:
    n = len(msg.ranges)
    return f"frame={msg.header.frame_id}, ranges={n}, min={msg.range_min:0.2f}, max={msg.range_max:0.2f}"


def _map_summary(msg: OccupancyGrid) -> str:
    w = msg.info.width
    h = msg.info.height
    res = msg.info.resolution
    return f"{w}x{h} @ {res:0.3f}m/cell"


def _odom_summary(msg: Odometry) -> str:
    p = msg.pose.pose.position
    return f"frame={msg.header.frame_id} child={msg.child_frame_id} x={p.x:0.2f} y={p.y:0.2f}"


def _quaternion_to_yaw(q) -> float:
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _get_robot_pose(odom_msg: Odometry) -> Tuple[float, float, float]:
    """Extract (x, y, yaw) from odometry message."""
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    yaw = _quaternion_to_yaw(q)
    return (p.x, p.y, yaw)


def _get_robot_velocity(odom_msg: Odometry) -> Tuple[float, float]:
    """Extract (linear_x, angular_z) from odometry message."""
    return (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z)


class MinidogWebNode(Node):
    def __init__(self, use_sim_time: bool, topics: Dict[str, str]):
        super().__init__("minidog_webapp")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)])

        self._lock = threading.Lock()
        self._state: Dict[str, TopicState] = {}

        # Publishers
        self.pub_autonomy = self.create_publisher(Bool, topics["autonomy_enabled"], 10)
        self.pub_manual = self.create_publisher(Twist, topics["cmd_vel_manual"], 10)

        # Subscriptions (monitoring)
        self._sub(Bool, topics["autonomy_enabled"], "autonomy_enabled", qos=QoSProfile(depth=10))
        self._sub(Twist, topics["cmd_vel"], "cmd_vel", qos=QoSProfile(depth=10))
        self._sub(Twist, topics["cmd_vel_manual"], "cmd_vel_manual", qos=QoSProfile(depth=10))
        self._sub(Twist, topics["cmd_vel_nav"], "cmd_vel_nav", qos=QoSProfile(depth=10))
        self._sub(LaserScan, topics["scan"], "scan", qos=qos_profile_sensor_data)
        self._sub(Odometry, topics["wheel_odom"], "wheel_odom", qos=qos_profile_sensor_data)

        # Map is often transient-local
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self._sub(OccupancyGrid, topics["map"], "map", qos=map_qos)

        self._sub(TFMessage, topics["tf"], "tf", qos=QoSProfile(depth=10))

        # Navigation and exploration status
        self._sub(GoalStatusArray, topics["navigate_to_pose_status"], "nav_status", qos=QoSProfile(depth=10))
        self._sub(Path, topics["plan"], "plan", qos=QoSProfile(depth=10))
        self._sub(Odometry, topics["odom"], "odom", qos=qos_profile_sensor_data)
        self._sub(String, topics["exploration_status"], "exploration_status", qos=QoSProfile(depth=10))

    def _sub(self, msg_type, topic: str, key: str, qos: QoSProfile):
        def cb(msg):
            with self._lock:
                stt = self._state.get(key) or TopicState()
                stt.last_recv_wall_time = _now()
                stt.last_msg = msg
                self._state[key] = stt

        self.create_subscription(msg_type, topic, cb, qos)

    def get_state_snapshot(self) -> Dict[str, TopicState]:
        with self._lock:
            return {k: TopicState(v.last_recv_wall_time, v.last_msg) for k, v in self._state.items()}

    # Commands
    def set_autonomy_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_autonomy.publish(msg)

    def publish_manual_twist(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.pub_manual.publish(msg)

    def publish_stop(self):
        self.pub_manual.publish(Twist())


@st.cache_resource
def get_ros_node() -> Tuple[MinidogWebNode, SingleThreadedExecutor]:
    rclpy.init(args=None)

    topics = {
        "autonomy_enabled": "/autonomy_enabled",
        "cmd_vel": "/cmd_vel",
        "cmd_vel_manual": "/cmd_vel_manual",
        "cmd_vel_nav": "/cmd_vel_nav",
        "scan": "/scan",
        "map": "/map",
        "tf": "/tf",
        "wheel_odom": "/wheel_odom",
        "odom": "/odom",
        "navigate_to_pose_status": "/navigate_to_pose/_action/status",
        "plan": "/plan",
        "exploration_status": "/exploration_status",
    }

    node = MinidogWebNode(use_sim_time=True, topics=topics)
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)

    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start()

    return node, exec_


st.set_page_config(page_title="minidog web", layout="wide")

# Auto-refresh UI (monitoring) every 1s
st_autorefresh(interval=1000, key="minidog_autorefresh_1s")

node, _exec = get_ros_node()
state = node.get_state_snapshot()

st.title("minidog web")

# Robot State Section
st.subheader("🤖 Robot State")
state_col1, state_col2, state_col3 = st.columns(3)

with state_col1:
    st.markdown("**Position & Orientation**")
    odom_state = state.get("odom")
    if odom_state and odom_state.last_msg:
        x, y, yaw = _get_robot_pose(odom_state.last_msg)
        st.metric("X", f"{x:.2f} m")
        st.metric("Y", f"{y:.2f} m")
        st.metric("Yaw", f"{math.degrees(yaw):.1f}°")
        st.caption(f"Age: {_fmt_age(_age_s(odom_state.last_recv_wall_time))}")
    else:
        st.warning("No odometry data")

with state_col2:
    st.markdown("**Velocity**")
    if odom_state and odom_state.last_msg:
        lin_x, ang_z = _get_robot_velocity(odom_state.last_msg)
        st.metric("Linear X", f"{lin_x:.2f} m/s")
        st.metric("Angular Z", f"{ang_z:.2f} rad/s")
        speed = abs(lin_x)
        if speed > 0.01:
            st.success("🟢 Moving")
        else:
            st.info("🟡 Stopped")
    else:
        st.warning("No velocity data")

with state_col3:
    st.markdown("**Navigation Status**")
    nav_status = state.get("nav_status")
    if nav_status and nav_status.last_msg and len(nav_status.last_msg.status_list) > 0:
        latest_status = nav_status.last_msg.status_list[-1]
        status_text = {
            0: "Unknown",
            1: "Accepted",
            2: "Executing",
            3: "Canceling",
            4: "Succeeded",
            5: "Canceled",
            6: "Aborted"
        }.get(latest_status.status, f"Status {latest_status.status}")

        if latest_status.status == 2:
            st.success(f"✅ {status_text}")
        elif latest_status.status == 4:
            st.info(f"🎯 {status_text}")
        elif latest_status.status == 6:
            st.error(f"❌ {status_text}")
        else:
            st.warning(f"⚠️ {status_text}")

        plan_state = state.get("plan")
        if plan_state and plan_state.last_msg:
            plan_len = len(plan_state.last_msg.poses)
            st.metric("Path points", plan_len)
    else:
        st.info("No active goal")

    # Autonomy status
    auto_state = state.get("autonomy_enabled")
    if auto_state and auto_state.last_msg:
        if auto_state.last_msg.data:
            st.success("🤖 Autonomy ON")
        else:
            st.warning("🎮 Manual mode")

st.divider()

# Exploration Stats Section
st.subheader("🗺️ Frontier Exploration")
explore_state = state.get("exploration_status")

if explore_state and explore_state.last_msg:
    try:
        stats = json.loads(explore_state.last_msg.data)

        exp_col1, exp_col2, exp_col3, exp_col4 = st.columns(4)

        with exp_col1:
            st.metric("Goals Sent", stats["goals_sent_total"])
            st.metric("Goals Succeeded", stats["goals_succeeded"])
            success_rate = stats["success_rate"] * 100
            st.metric("Success Rate", f"{success_rate:.1f}%")

        with exp_col2:
            st.metric("Frontier Clusters", stats["frontier_clusters"])
            st.metric("Blocked Goals", stats["blocked_goals_count"])
            if stats["goal_distance"]:
                st.metric("Goal Distance", f"{stats['goal_distance']:.2f}m")
            else:
                st.metric("Goal Distance", "—")

        with exp_col3:
            st.metric("Consecutive Aborts", stats["consecutive_aborts"])
            progress = f"{stats['no_frontier_streak']}/{stats['done_threshold']}"
            st.metric("No Frontier Streak", progress)

        with exp_col4:
            # Status indicators
            if stats["exploration_complete"]:
                st.success("✅ Exploration Complete!")
            elif stats["goal_in_flight"]:
                st.info("🎯 Goal In Flight")
            elif not stats["enabled"]:
                st.warning("⏸️ Autonomy Disabled")
            else:
                st.success("🔍 Exploring")

            if stats["unstick_active"]:
                st.warning("⚠️ Unsticking (reversing)")
            if stats["costmaps_clearing"]:
                st.warning("🔄 Clearing costmaps")

        st.caption(f"Last update: {_fmt_age(_age_s(explore_state.last_recv_wall_time))}")

    except json.JSONDecodeError as e:
        st.error(f"Failed to parse exploration status: {e}")
else:
    st.info("No exploration status available")

st.divider()

col1, col2 = st.columns([1, 2], gap="large")

with col1:
    st.subheader("Operation")

    # Keep UI state in session_state so we can reset sliders/toggle.
    if "autonomy_enabled_ui" not in st.session_state:
        st.session_state.autonomy_enabled_ui = False
    if "manual_lin_x" not in st.session_state:
        st.session_state.manual_lin_x = 0.0
    if "manual_ang_z" not in st.session_state:
        st.session_state.manual_ang_z = 0.0
    if "last_published_autonomy" not in st.session_state:
        st.session_state.last_published_autonomy = None

    # Observe last received autonomy state (for user feedback, not as UI source of truth).
    last_auto = state.get("autonomy_enabled")
    observed_auto = None
    if last_auto and isinstance(last_auto.last_msg, Bool):
        observed_auto = bool(last_auto.last_msg.data)
    st.caption(
        f"/autonomy_enabled last seen: {_fmt_age(_age_s(last_auto.last_recv_wall_time) if last_auto else None)}"
    )
    st.write(f"Autonomy (observed): **{observed_auto if observed_auto is not None else '—'}**")

    # Callbacks run BEFORE the next script rerun, so they can safely modify
    # widget-bound session_state keys.
    def _on_autonomy_toggle():
        enabled = st.session_state.autonomy_enabled_ui
        if enabled:
            node.publish_stop()
            node.set_autonomy_enabled(True)
        else:
            node.set_autonomy_enabled(False)
            node.publish_stop()
            st.session_state.manual_lin_x = 0.0
            st.session_state.manual_ang_z = 0.0
        st.session_state.last_published_autonomy = enabled

    def _on_emergency_stop():
        node.set_autonomy_enabled(False)
        node.publish_stop()
        st.session_state.autonomy_enabled_ui = False
        st.session_state.last_published_autonomy = False
        st.session_state.manual_lin_x = 0.0
        st.session_state.manual_ang_z = 0.0

    # Single autonomy toggle
    autonomy_enabled = st.toggle(
        "Enable autonomy", key="autonomy_enabled_ui", on_change=_on_autonomy_toggle
    )

    # Sync tracking on first run.
    if st.session_state.last_published_autonomy is None:
        st.session_state.last_published_autonomy = autonomy_enabled

    # Emergency stop: switches to manual, sends zero velocity, resets UI.
    st.button("EMERGENCY STOP", on_click=_on_emergency_stop, type="primary")

    st.divider()
    st.subheader("Manual cmd_vel")
    lin_x = st.slider(
        "linear.x",
        min_value=-1.0,
        max_value=1.0,
        step=0.01,
        key="manual_lin_x",
    )
    ang_z = st.slider(
        "angular.z",
        min_value=-2.0,
        max_value=2.0,
        step=0.01,
        key="manual_ang_z",
    )
    if st.button("Send /cmd_vel_manual"):
        node.publish_manual_twist(lin_x, ang_z)

with col2:
    st.subheader("Monitoring")
    st.caption("OK = message received recently (within a per-topic age threshold).")

    # (key, display_name, ok_age_s, formatter)
    monitors = [
        ("scan", "/scan", 0.5, _scan_summary),
        ("map", "/map", 5.0, _map_summary),
        ("tf", "/tf", 0.5, lambda m: f"transforms={len(m.transforms)}"),
        ("odom", "/odom", 1.0, _odom_summary),
        ("wheel_odom", "/wheel_odom", 1.0, _odom_summary),
        ("cmd_vel", "/cmd_vel", 1.0, _twist_summary),
        ("cmd_vel_manual", "/cmd_vel_manual", 1.0, _twist_summary),
        ("cmd_vel_nav", "/cmd_vel_nav", 1.0, _twist_summary),
        ("plan", "/plan", 2.0, lambda m: f"waypoints={len(m.poses)}"),
        ("nav_status", "/navigate_to_pose/_action/status", 2.0, lambda m: f"goals={len(m.status_list)}"),
        ("autonomy_enabled", "/autonomy_enabled", 1.0, lambda m: f"data={bool(m.data)}"),
        ("exploration_status", "/exploration_status", 3.0, lambda m: f"data_len={len(m.data)}"),
    ]

    rows = []
    for key, name, ok_age, fmt in monitors:
        stt = state.get(key)
        age = _age_s(stt.last_recv_wall_time) if stt else None
        ok = (age is not None) and (age <= ok_age)
        summary = "—"
        if stt and stt.last_msg is not None:
            try:
                summary = fmt(stt.last_msg)
            except Exception as e:
                summary = f"(format error: {e})"
        rows.append(
            {
                "topic": name,
                "ok": ok,
                "age": _fmt_age(age),
                "details": summary,
            }
        )

    st.dataframe(rows, use_container_width=True, hide_index=True)

st.caption(
    "Tip: run this with `streamlit run <...>/app.py`. Make sure you sourced ROS2 + your workspace so rclpy can find your packages."
)


