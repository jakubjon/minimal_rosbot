#!/usr/bin/env python3
"""Data relay for GO2 rosbag replay.

Subscribes to namespaced GO2 topics and republishes on standard topics
expected by the SLAM/Nav2/exploration pipeline.

Applies corrections for known GO2 data quirks:
1. 90-degree odometry position rotation (GO2 X=left, Y=forward)
2. 180-degree LaserScan rotation (scan/pointcloud frame mismatch)
3. Self-referential TF filtering (os_sensor -> os_sensor)
4. odom -> base_link TF publishing from odometry messages
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import copy


class DataRelay(Node):
    def __init__(self):
        super().__init__("data_relay")

        self.declare_parameter("namespace", "go2_unit_27778")
        namespace = self.get_parameter("namespace").value

        # QoS profiles
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        dynamic_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_reliable_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Publishers — standard topics for SLAM/Nav2
        self.tf_pub = self.create_publisher(TFMessage, "/tf", dynamic_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", sensor_reliable_qos)
        self.points_pub = self.create_publisher(PointCloud2, "/points", sensor_reliable_qos)

        # Subscribers — namespaced GO2 topics
        self.create_subscription(
            TFMessage, f"/{namespace}/tf", self.tf_callback, dynamic_qos
        )
        self.create_subscription(
            TFMessage, f"/{namespace}/tf_static", self.tf_static_callback, static_qos
        )
        self.create_subscription(
            TFMessage, f"/{namespace}/ouster/tf_static", self.tf_static_callback, static_qos
        )
        self.create_subscription(
            LaserScan, f"/{namespace}/ouster/scan", self.scan_callback, sensor_qos
        )
        self.create_subscription(
            Odometry, f"/{namespace}/base/odom", self.odom_callback, sensor_reliable_qos
        )
        self.create_subscription(
            PointCloud2, f"/{namespace}/ouster/points", self.points_callback, sensor_reliable_qos
        )

        # Counters for first-message logging
        self.tf_count = 0
        self.tf_static_count = 0
        self.scan_count = 0
        self.odom_count = 0
        self.points_count = 0

        self.get_logger().info(f"Data relay started for namespace: {namespace}")
        self.get_logger().info("Corrections: 90° odom rotation, 180° scan rotation, odom->base_link TF")

    def tf_callback(self, msg):
        """Relay dynamic TF with current timestamps."""
        now = self.get_clock().now().to_msg()
        new_msg = TFMessage()
        for transform in msg.transforms:
            new_transform = copy.deepcopy(transform)
            new_transform.header.stamp = now
            new_msg.transforms.append(new_transform)
        self.tf_pub.publish(new_msg)
        self.tf_count += 1
        if self.tf_count == 1:
            frames = [t.child_frame_id for t in msg.transforms]
            self.get_logger().info(f"First TF - frames: {frames}")

    def tf_static_callback(self, msg):
        """Relay static TF, filtering self-referential transforms."""
        now = self.get_clock().now().to_msg()
        new_msg = TFMessage()
        for transform in msg.transforms:
            # Filter self-referential os_sensor -> os_sensor (180° yaw hack)
            if transform.header.frame_id == transform.child_frame_id:
                if self.tf_static_count == 0:
                    self.get_logger().info(
                        f"Filtering self-referential TF: {transform.header.frame_id}"
                    )
                continue
            new_transform = copy.deepcopy(transform)
            new_transform.header.stamp = now
            new_msg.transforms.append(new_transform)
        if new_msg.transforms:
            self.tf_static_pub.publish(new_msg)
        self.tf_static_count += 1

    def scan_callback(self, msg):
        """Relay LaserScan with 180° rotation correction."""
        new_msg = copy.deepcopy(msg)
        new_msg.header.stamp = self.get_clock().now().to_msg()

        # Rotate scan by 180° — Ouster LaserScan angle=0 is opposite to PointCloud2 X-axis
        num_ranges = len(new_msg.ranges)
        if num_ranges > 0:
            half = num_ranges // 2
            new_msg.ranges = list(new_msg.ranges[half:]) + list(new_msg.ranges[:half])
            if len(new_msg.intensities) == num_ranges:
                new_msg.intensities = list(new_msg.intensities[half:]) + list(new_msg.intensities[:half])

        self.scan_pub.publish(new_msg)
        self.scan_count += 1
        if self.scan_count == 1:
            self.get_logger().info(f"First scan - frame: {msg.header.frame_id}, rotated 180°")

    def odom_callback(self, msg):
        """Relay odometry with 90° position rotation and publish odom->base_link TF."""
        now = self.get_clock().now().to_msg()

        # GO2 odom has 90° rotation: X=left, Y=forward (non-standard)
        # Correct: new_x = y, new_y = -x
        orig_x = msg.pose.pose.position.x
        orig_y = msg.pose.pose.position.y
        corrected_x = orig_y
        corrected_y = -orig_x

        # Publish corrected odometry
        new_msg = copy.deepcopy(msg)
        new_msg.header.stamp = now
        new_msg.pose.pose.position.x = corrected_x
        new_msg.pose.pose.position.y = corrected_y
        self.odom_pub.publish(new_msg)

        # Publish odom -> base_link TF (not in bag, must be derived)
        odom_tf = TransformStamped()
        odom_tf.header.stamp = now
        odom_tf.header.frame_id = msg.header.frame_id       # "odom"
        odom_tf.child_frame_id = msg.child_frame_id          # "base_link"
        odom_tf.transform.translation.x = corrected_x
        odom_tf.transform.translation.y = corrected_y
        odom_tf.transform.translation.z = msg.pose.pose.position.z
        odom_tf.transform.rotation = msg.pose.pose.orientation

        tf_msg = TFMessage()
        tf_msg.transforms.append(odom_tf)
        self.tf_pub.publish(tf_msg)

        self.odom_count += 1
        if self.odom_count == 1:
            self.get_logger().info(
                f"First odom - {msg.header.frame_id} -> {msg.child_frame_id}, "
                f"position rotation applied"
            )


    def points_callback(self, msg):
        """Relay PointCloud2 with updated timestamp."""
        new_msg = copy.deepcopy(msg)
        new_msg.header.stamp = self.get_clock().now().to_msg()
        self.points_pub.publish(new_msg)
        self.points_count += 1
        if self.points_count == 1:
            self.get_logger().info(
                f"First PointCloud2 - frame: {msg.header.frame_id}, "
                f"width={msg.width}, height={msg.height}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DataRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
