#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class RRTNode(Node):
    def __init__(self):
        super().__init__("rrt_node")
        self.get_logger().info("Python rrt_node has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()