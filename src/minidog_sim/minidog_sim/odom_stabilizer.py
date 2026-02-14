#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from copy import deepcopy

class OdomStabilizer(Node):
    def __init__(self):
        super().__init__('odom_stabilizer')
        
        # Parameters
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.freq = self.declare_parameter('freq', 20.0).value
        
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
        
        # Timer to ensure constant publication rate
        self.create_timer(1.0/self.freq, self.publish_loop)

    def odom_callback(self, msg):
        self.last_valid_odom = msg
        self.last_received_time = self.get_clock().now()

    def publish_loop(self):
        if self.last_valid_odom is None:
            return

        current_time = self.get_clock().now()

        # Deep copy to avoid mutating the stored message
        out_msg = deepcopy(self.last_valid_odom)

        # Override header time to be current (Keep Alive strategy)
        out_msg.header.stamp = current_time.to_msg()
        out_msg.header.frame_id = self.odom_frame
        out_msg.child_frame_id = self.base_frame

        self.pub.publish(out_msg)
        
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = out_msg.pose.pose.position.x
            t.transform.translation.y = out_msg.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = out_msg.pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
