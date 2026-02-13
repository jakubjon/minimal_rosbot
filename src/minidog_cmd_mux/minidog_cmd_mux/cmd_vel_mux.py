import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("minidog_cmd_vel_mux")

        self.declare_parameter("manual_topic", "/cmd_vel_manual")
        self.declare_parameter("auto_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("enable_topic", "/autonomy_enabled")
        self.declare_parameter("auto_timeout_sec", 0.5)
        self.declare_parameter("manual_override", True)
        self.declare_parameter("manual_override_epsilon", 1e-3)
        self.declare_parameter("manual_override_hold_sec", 0.5)

        manual_topic = self.get_parameter("manual_topic").get_parameter_value().string_value
        auto_topic = self.get_parameter("auto_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        enable_topic = self.get_parameter("enable_topic").get_parameter_value().string_value
        self.auto_timeout_sec = (
            self.get_parameter("auto_timeout_sec").get_parameter_value().double_value
        )
        self.manual_override = (
            self.get_parameter("manual_override").get_parameter_value().bool_value
        )
        self.manual_override_epsilon = (
            self.get_parameter("manual_override_epsilon").get_parameter_value().double_value
        )
        self.manual_override_hold_sec = (
            self.get_parameter("manual_override_hold_sec").get_parameter_value().double_value
        )

        # Start with autonomy enabled so the robot explores immediately.
        self.declare_parameter("start_enabled", True)
        self.autonomy_enabled = self.get_parameter("start_enabled").get_parameter_value().bool_value
        self.last_auto_msg_time = None
        self.last_auto_cmd = Twist()
        self.last_manual_msg_time = None
        self.last_manual_mag = 0.0

        self.pub = self.create_publisher(Twist, output_topic, 10)

        self.sub_enable = self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.sub_manual = self.create_subscription(Twist, manual_topic, self._on_manual, 10)
        self.sub_auto = self.create_subscription(Twist, auto_topic, self._on_auto, 10)

        # Safety timer: publish stop if autonomy is enabled but Nav2 stops sending commands.
        self._auto_timeout_sent_stop = False
        self.create_timer(0.25, self._check_auto_timeout)

        self.get_logger().info(
            f"cmd_vel_mux: manual={manual_topic} auto={auto_topic} out={output_topic} enable={enable_topic}"
        )

    def _on_enable(self, msg: Bool):
        self.autonomy_enabled = bool(msg.data)
        if not self.autonomy_enabled:
            self._auto_timeout_sent_stop = False

    def _check_auto_timeout(self):
        """If autonomy is enabled and Nav2 stops sending, publish a stop for safety."""
        if not self.autonomy_enabled or self.last_auto_msg_time is None:
            return
        age = (self.get_clock().now() - self.last_auto_msg_time).nanoseconds / 1e9
        if age > self.auto_timeout_sec and not self._auto_timeout_sent_stop:
            self.pub.publish(Twist())  # zero velocity
            self._auto_timeout_sent_stop = True
            self.get_logger().warn(
                f"No auto cmd_vel for {age:.1f}s (timeout={self.auto_timeout_sec}s); sent stop"
            )

    def _on_auto(self, msg: Twist):
        self.last_auto_cmd = msg
        self.last_auto_msg_time = self.get_clock().now()
        self._auto_timeout_sent_stop = False
        if self.autonomy_enabled:
            if self.manual_override and (self.last_manual_msg_time is not None):
                dt = (self.get_clock().now() - self.last_manual_msg_time).nanoseconds / 1e9
                if (dt < self.manual_override_hold_sec) and (
                    self.last_manual_mag > self.manual_override_epsilon
                ):
                    # Manual override is active; ignore autonomous cmd_vel.
                    return
            self.pub.publish(msg)

    def _on_manual(self, msg: Twist):
        self.last_manual_msg_time = self.get_clock().now()
        self.last_manual_mag = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)

        # Manual mode: always forward.
        if not self.autonomy_enabled:
            self.pub.publish(msg)
            return

        # Autonomy mode: allow manual override (e.g. bootstrap mapping / safety).
        if self.manual_override and (self.last_manual_mag > self.manual_override_epsilon):
            self.pub.publish(msg)
            return


def main():
    rclpy.init()
    node = CmdVelMux()
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


