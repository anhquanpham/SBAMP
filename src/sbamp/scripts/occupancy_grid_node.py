#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")
        self.get_logger().info("Python occupancy_grid_node has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()