#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Robot parameters (adjust to match your robot)
        self.wheel_radius = 0.0325  # in meters
        self.wheel_base = 0.34     # distance between left and right wheel centers
        self.ticks_per_rev = 1000   # encoder resolution
        self.gear_ratio = 1.0      # if any
        self.encoder_counts_per_meter = (self.ticks_per_rev * self.gear_ratio) / (2 * math.pi * self.wheel_radius)

        # State
        self.prev_encoder_counts = [0, 0, 0, 0]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ROS 2 interfaces
        self.encoder_sub = self.create_subscription(Int32MultiArray, 'encoder_counts', self.encoder_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Odometry node started.")

    def encoder_callback(self, msg):
        # Get current time and calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if len(msg.data) != 4:
            self.get_logger().error("Invalid encoder message length")
            return

        # Average left and right encoders (1,3) are left, (2,4) are right
        left_enc = (msg.data[0] + msg.data[2]) / 2.0
        right_enc = (msg.data[1] + msg.data[3]) / 2.0

        prev_left = (self.prev_encoder_counts[0] + self.prev_encoder_counts[2]) / 2.0
        prev_right = (self.prev_encoder_counts[1] + self.prev_encoder_counts[3]) / 2.0

        d_left = -(left_enc - prev_left) / self.encoder_counts_per_meter
        d_right = (right_enc - prev_right) / self.encoder_counts_per_meter

        self.prev_encoder_counts = msg.data

        # Compute displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = -(d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation as quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set dummy velocity (optional, can be improved)
        odom_msg.twist.twist.linear.x = d_center / dt
        odom_msg.twist.twist.angular.z = d_theta / dt

        self.odom_pub.publish(odom_msg)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
