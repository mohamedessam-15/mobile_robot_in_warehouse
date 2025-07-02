#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('encoder_speed_node')

        # Parameters — set appropriately for your robot
        self.encoder_resolution = 1000       # counts per revolution
        self.wheel_radius = 0.03            # in meters
        self.wheel_circumference = 2 * math.pi * self.wheel_radius

        self.last_encoder = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Int32, 'encoder_counts', self.encoder_callback, 10)

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        if self.last_encoder is None:
            self.last_encoder = msg.data
            self.last_time = current_time
            return

        delta_counts = msg.data - self.last_encoder
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds
        self.last_encoder = msg.data
        self.last_time = current_time

        # Distance moved
        revs = delta_counts / self.encoder_resolution
        distance = revs * self.wheel_circumference

        # Assuming straight motion
        self.x += distance
        vx = distance / dt if dt > 0 else 0.0

        # === Publish Odometry ===
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0  # No rotation for now

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        # === Publish TF ===
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # No rotation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
