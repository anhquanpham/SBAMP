#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

import numpy as np
from transforms3d.euler import quat2euler

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")
        self.get_logger().info("Python occupancy_grid_node has been started.")

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/ego_racecar/odom')
        self.declare_parameter('original_map_topic', '/map')

        self.declare_parameter('map_height', 713)
        self.declare_parameter('map_width', 727)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_origin', (-20.2, -5.68))

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        original_map_topic = self.get_parameter('original_map_topic').get_parameter_value().string_value

        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_origin = self.get_parameter('map_origin').get_parameter_value().double_array_value
       
        # Subscribers
        
        map_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # NOTE: self.occupancy_grid is defined and set inside map_callback
        self.map_subscriber_ = self.create_subscription(OccupancyGrid, original_map_topic, self.map_callback, map_qos_profile)


        qos_profile = QoSProfile(depth=10)

        # NOTE: self.cur_pos and self.cur_yaw is defined and set inside pose_callback
        self.pose_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, qos_profile)

        # NOTE: self.occupancy_grid is updated and published inside scan_callback
        self.scan_subscriber_ = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile)         


    def scan_callback(self, msg):
        # Process the LaserScan message
        self.get_logger().info(f"Received scan data with {len(msg.ranges)} ranges.")
        

    def pose_callback(self, pose_msg):
        # Process the Odometry message
        self.cur_pos = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)
        self.cur_yaw = quat2euler([pose_msg.pose.pose.orientation.x,
                                          pose_msg.pose.pose.orientation.y,
                                          pose_msg.pose.pose.orientation.z,
                                          pose_msg.pose.pose.orientation.w])[2]

        # self.get_logger().info(f"Received pose data: {self.cur_pos}, yaw: {self.cur_yaw}")

    def map_callback(self, msg):
        # Process the map message

        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        self.occupancy_grid = np.array(msg.data).reshape((self.map_height, self.map_width))

        self.get_logger().info(f"Received map and occupancy_grid set: \n"
                               f"height: {self.map_height}, width: {self.map_width},\n"
                               f"resolution: {self.map_resolution}, \n"
                               f"origin: {self.map_origin}"
                               )

       

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()