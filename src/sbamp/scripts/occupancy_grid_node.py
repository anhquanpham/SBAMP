#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")
        self.get_logger().info("Python occupancy_grid_node has been started.")

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/ego_racecar/odom')

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        qos_profile = QoSProfile(depth=10)

        self.scan_subscriber_ = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile) 
        self.pose_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, qos_profile)

    def scan_callback(self, msg):
        # Process the LaserScan message
        self.get_logger().info(f"Received scan data with {len(msg.ranges)} ranges.")
        

    def pose_callback(self, msg):
        # Process the Odometry message
        self.get_logger().info(f"Received pose data: position ({msg.pose.pose.position.x}, {msg.pose.pose.position.y}), orientation ({msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}, {msg.pose.pose.orientation.w})")

       

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()