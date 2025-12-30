import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

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
        self._sub(LaserScan, topics["scan"], "scan", qos=QoSProfile(depth=10))
        self._sub(Odometry, topics["wheel_odom"], "wheel_odom", qos=QoSProfile(depth=10))

        # Map is often transient-local
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self._sub(OccupancyGrid, topics["map"], "map", qos=map_qos)

        self._sub(TFMessage, topics["tf"], "tf", qos=QoSProfile(depth=10))

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

    # Single autonomy toggle
    autonomy_enabled = st.toggle("Enable autonomy", key="autonomy_enabled_ui")

    # Publish on transitions only.
    if st.session_state.last_published_autonomy is None:
        st.session_state.last_published_autonomy = autonomy_enabled
    elif autonomy_enabled != st.session_state.last_published_autonomy:
        # ON: send zero manual once, then set autonomy true
        if autonomy_enabled:
            node.publish_stop()
            node.set_autonomy_enabled(True)
        # OFF: set autonomy false, send zero manual, reset sliders
        else:
            node.set_autonomy_enabled(False)
            node.publish_stop()
            st.session_state.manual_lin_x = 0.0
            st.session_state.manual_ang_z = 0.0

        st.session_state.last_published_autonomy = autonomy_enabled

    # Stop button: switches to manual, sends 0, resets sliders, sets toggle OFF
    if st.button("STOP"):
        node.set_autonomy_enabled(False)
        node.publish_stop()
        st.session_state.autonomy_enabled_ui = False
        st.session_state.last_published_autonomy = False
        st.session_state.manual_lin_x = 0.0
        st.session_state.manual_ang_z = 0.0

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
        ("cmd_vel_manual", "/cmd_vel_manual", 1.0, _twist_summary),
        ("cmd_vel_nav", "/cmd_vel_nav", 1.0, _twist_summary),
        ("cmd_vel", "/cmd_vel", 1.0, _twist_summary),
        ("wheel_odom", "/wheel_odom", 1.0, _odom_summary),
        ("autonomy_enabled", "/autonomy_enabled", 1.0, lambda m: f"data={bool(m.data)}"),
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


