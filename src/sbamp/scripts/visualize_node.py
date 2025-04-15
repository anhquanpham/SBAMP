#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

class VisualizeNode(Node):
    def __init__(self):
        super().__init__("visualize_node")
        self.get_logger().info("Python visualize_node has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()