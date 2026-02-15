#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomStabilizer(Node):
    def __init__(self):
        super().__init__('odom_stabilizer')

        # Parameters
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.freq = self.declare_parameter('freq', 20.0).value
        self.max_stale_sec = self.declare_parameter('max_stale_sec', 0.5).value

        # Subscribe to raw RF2O output (no TF from RF2O)
        self.sub = self.create_subscription(
            Odometry,
            'odom_rf2o',
            self.odom_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Publishers
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_valid_odom = None
        self.last_received_time = self.get_clock().now()
        self._stale_warned = False

        # Timer to ensure constant publication rate
        self.create_timer(1.0/self.freq, self.publish_loop)

    def odom_callback(self, msg):
        self.last_valid_odom = msg
        self.last_received_time = self.get_clock().now()
        self._stale_warned = False

    def publish_loop(self):
        if self.last_valid_odom is None:
            return

        current_time = self.get_clock().now()

        # Check staleness: don't republish stale data with fresh timestamps
        age_sec = (current_time - self.last_received_time).nanoseconds / 1e9
        if age_sec > self.max_stale_sec:
            if not self._stale_warned:
                self.get_logger().warn(
                    f"RF2O odometry stale ({age_sec:.2f}s > {self.max_stale_sec}s); "
                    f"skipping publish to let TF tree age naturally"
                )
                self._stale_warned = True
            return

        # Build output message by copying needed fields (avoid expensive deepcopy)
        src = self.last_valid_odom
        out_msg = Odometry()
        out_msg.header.stamp = current_time.to_msg()
        out_msg.header.frame_id = self.odom_frame
        out_msg.child_frame_id = self.base_frame
        out_msg.pose.pose.position.x = src.pose.pose.position.x
        out_msg.pose.pose.position.y = src.pose.pose.position.y
        out_msg.pose.pose.position.z = src.pose.pose.position.z
        out_msg.pose.pose.orientation = src.pose.pose.orientation
        out_msg.twist.twist.linear = src.twist.twist.linear
        out_msg.twist.twist.angular = src.twist.twist.angular

        self.pub.publish(out_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = src.pose.pose.position.x
            t.transform.translation.y = src.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = src.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
