#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sbamp.ds_opt_py.sbamp_header import demo

class NextWaypoint(Node):
    def __init__(self):
        super().__init__("next_wp_node")
        self.get_logger().info("Python next_wp_node has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = NextWaypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()