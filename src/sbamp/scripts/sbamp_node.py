#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sbamp.ds_opt_py.sbamp_header import demo

class SBAMPNode(Node):
    def __init__(self):
        super().__init__("sbamp_node")
        self.get_logger().info("Python sbamp_node has been started.")

        # Call the demo function from sbamp_header.py
        demo_result = demo()
        self.get_logger().info(f"Demo result: {demo_result}")

def main(args=None):
    rclpy.init(args=args)
    node = SBAMPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()