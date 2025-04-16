#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry

import numpy as np
from transforms3d.euler import quat2euler

class RRTNode(Node):
    def __init__(self):
        super().__init__("rrt_node")
        self.get_logger().info("Python rrt_node has been started.")

        self.declare_parameter('pose_topic', '/ego_racecar/odom')
        self.declare_parameter('occupancy_grid_topic', '/occupancy_grid')

        self.declare_parameter('map_height', 713)
        self.declare_parameter('map_width', 727)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_origin', (-20.2, -5.68))

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').get_parameter_value().string_value

        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_origin = self.get_parameter('map_origin').get_parameter_value().double_array_value


        occupancy_grid_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )  

        # NOTE: self.occupancy_grid is defined and set inside occupancy_grid_callback
        self.occupancy_grid_subscriber_ = self.create_subscription(OccupancyGrid, occupancy_grid_topic, self.occupancy_grid_callback, occupancy_grid_qos_profile)


        qos_profile = QoSProfile(depth=10)

        # NOTE: self.cur_pos and self.cur_yaw is defined and set inside pose_callback
        self.pose_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, qos_profile)

    def occupancy_grid_callback(self, occ_grid_msg):
        
        self.map_height = occ_grid_msg.info.height
        self.map_width = occ_grid_msg.info.width
        self.map_resolution = occ_grid_msg.info.resolution
        self.map_origin = (occ_grid_msg.info.origin.position.x, occ_grid_msg.info.origin.position.y)

        self.occupancy_grid = np.array(occ_grid_msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))

        
        # self.get_logger().info(f"Received map and occupancy_grid set: \n"
        #                        f"height: {self.map_height}, width: {self.map_width},\n"
        #                        f"resolution: {self.map_resolution}, \n"
        #                        f"origin: {self.map_origin}"
        #                        )

    def pose_callback(self, pose_msg):
        # Process the Odometry message
        self.cur_pos = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)
        self.cur_yaw = quat2euler([pose_msg.pose.pose.orientation.w,
                                          pose_msg.pose.pose.orientation.x,
                                          pose_msg.pose.pose.orientation.y,
                                          pose_msg.pose.pose.orientation.z])[2]
        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()