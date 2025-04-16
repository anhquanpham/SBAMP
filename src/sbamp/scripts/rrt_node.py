#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class RRTNode(Node):
    def __init__(self):
        super().__init__("rrt_node")
        self.get_logger().info("Python rrt_node has been started.")

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/ego_racecar/odom')
        self.declare_parameter('occupancy_grid_topic', '/occupancy_grid')


        occupancy_grid_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ADD CODE TO GET OCCUPANCY GRID AND POSE HERE

    

def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()