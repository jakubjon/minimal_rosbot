#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

class ScanSafetyFilter(Node):
    def __init__(self):
        super().__init__('scan_safety_filter')

        # Subscribe to raw scan
        self.sub = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Publish safe scan
        self.pub = self.create_publisher(
            LaserScan,
            'scan_safe',
            rclpy.qos.qos_profile_sensor_data
        )

        self.last_scan_time = None
        self.min_dt_ns = 1  # Nanoseconds (1ns minimum)

    def scan_callback(self, msg):
        # Use rclpy.time.Time for proper timestamp handling
        current_time = Time.from_msg(msg.header.stamp)

        if self.last_scan_time is not None:
            dt_ns = abs(current_time.nanoseconds - self.last_scan_time.nanoseconds)
            if dt_ns < self.min_dt_ns:
                self.get_logger().warn(
                    f"Checking for zero time increment. Dropping scan. dt={dt_ns}ns"
                )
                return

        self.last_scan_time = current_time
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanSafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
