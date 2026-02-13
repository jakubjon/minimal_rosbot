#!/usr/bin/env python3
"""Odometry TF publisher for the minidog robot.

Supports two modes (set via ``mode`` parameter):

**wheel** (default)
    Subscribes to a wheel-odometry topic (``/wheel_odom``) published by the
    Gazebo bridge and re-publishes it as:
    * ``/odom`` (``nav_msgs/Odometry``)
    * TF: ``minidog/odom`` -> ``minidog/base_footprint``

    This bypasses the ``ros_gz_bridge`` TF bridging (``Pose_V``), which
    produces a corrupt/flat TF tree that causes SLAM and Nav2 failures.

**scan**
    Publishes a static identity ``odom -> base_footprint`` TF and a
    zero-velocity ``/odom`` message.  SLAM (``slam_toolbox``) provides
    all localization via its ``map -> odom`` TF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros


class ScanOdom(Node):
    def __init__(self):
        super().__init__("scan_odom")

        # Parameters
        self.declare_parameter("mode", "wheel")  # "wheel" or "scan"
        self.declare_parameter("odom_frame_id", "minidog/odom")
        self.declare_parameter("base_frame_id", "minidog/base_footprint")
        self.declare_parameter("laser_scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("publish_tf", True)

        self.mode = self.get_parameter("mode").value
        self.odom_frame = self.get_parameter("odom_frame_id").value
        self.base_frame = self.get_parameter("base_frame_id").value
        scan_topic = self.get_parameter("laser_scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        wheel_odom_topic = self.get_parameter("wheel_odom_topic").value
        self.do_tf = self.get_parameter("publish_tf").value

        # TF broadcaster
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        # Odom publisher (output)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        if self.mode == "wheel":
            # ---- Wheel-odom mode ----
            # Subscribe to the Gazebo wheel-odometry topic and republish
            # as proper TF + /odom.
            sensor_qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.wheel_sub = self.create_subscription(
                Odometry, wheel_odom_topic, self._wheel_odom_cb, sensor_qos
            )
            self.get_logger().info(
                f"scan_odom (wheel mode): {wheel_odom_topic} -> "
                f"TF {self.odom_frame}->{self.base_frame} + {odom_topic}"
            )
        else:
            # ---- Scan / SLAM-delegated mode ----
            # Publish identity odom->base TF; SLAM provides map->odom.
            sensor_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.scan_sub = self.create_subscription(
                LaserScan, scan_topic, self._scan_cb, sensor_qos
            )
            # Fallback timer for when scans are delayed
            self.create_timer(0.1, self._timer_cb)
            self._last_stamp = None
            self.get_logger().info(
                f"scan_odom (SLAM-delegated): {self.odom_frame}->{self.base_frame}, "
                f"odom={odom_topic}"
            )

    # -----------------------------------------------------------------
    # Wheel-odom mode
    # -----------------------------------------------------------------
    def _wheel_odom_cb(self, msg: Odometry):
        """Forward Gazebo wheel odometry as proper TF + /odom."""
        # Re-publish as /odom with correct frame IDs
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        # Broadcast TF: odom -> base_footprint
        if self.do_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_br.sendTransform(t)

    # -----------------------------------------------------------------
    # Scan / SLAM-delegated mode
    # -----------------------------------------------------------------
    def _publish_identity(self, stamp):
        """Publish identity odom TF and zero-velocity odom message."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        if self.do_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.rotation.w = 1.0
            self.tf_br.sendTransform(t)

    def _scan_cb(self, msg: LaserScan):
        self._last_stamp = msg.header.stamp
        self._publish_identity(msg.header.stamp)

    def _timer_cb(self):
        if self._last_stamp is None:
            self._publish_identity(self.get_clock().now().to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = ScanOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
