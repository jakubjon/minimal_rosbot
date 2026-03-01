#!/usr/bin/env python3
"""Fix Gazebo-bridged IMU: set frame_id to base_link and inject covariances.

ros_gz_bridge sets frame_id to the sensor's Gazebo scoped name
(e.g. 'diffbot/base_link/imu_sensor') and leaves all covariances=0.
robot_localization EKF requires a valid frame and non-zero covariances.
"""
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFixup(Node):
    def __init__(self):
        super().__init__('imu_fixup')

        # Diagonal covariances matching the SDF noise stddev values
        gyro_std  = 0.002   # rad/s  — matches SDF angular_velocity noise
        accel_std = 0.04    # m/s²   — matches SDF linear_acceleration noise
        orient_std_xy = 0.01  # rad — roll/pitch (small, 2D ground robot)
        orient_std_z  = 0.1   # rad — yaw (less certain from Gazebo integration)

        def diag3(v): return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]

        self._orient_cov = [orient_std_xy**2, 0.0, 0.0,
                            0.0, orient_std_xy**2, 0.0,
                            0.0, 0.0, orient_std_z**2]
        self._gyro_cov  = diag3(gyro_std**2)
        self._accel_cov = diag3(accel_std**2)

        qos = rclpy.qos.qos_profile_sensor_data
        self.pub = self.create_publisher(Imu, 'imu/data', qos)
        self.create_subscription(Imu, 'imu/bridge_raw', self._cb, qos)

    def _cb(self, msg):
        out = copy.copy(msg)
        out.header.frame_id = 'base_link'
        for i in range(9):
            out.orientation_covariance[i]         = self._orient_cov[i]
            out.angular_velocity_covariance[i]    = self._gyro_cov[i]
            out.linear_acceleration_covariance[i] = self._accel_cov[i]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFixup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
