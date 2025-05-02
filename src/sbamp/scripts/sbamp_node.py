#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sbamp.ds_opt_py.lpv_opt.optimize_lpv_ds_from_data import optimize_lpv_ds_from_data
from sbamp.ds_opt_py.lpv_opt.lpv_ds import lpv_ds
    
from nav_msgs.msg import Path

import numpy as np

from dataclasses import dataclass

@dataclass
class EstimationOptions:
    # Required parameters
    type: int            # 0=Non-parametric, 1=?, 2=?
    do_plots: bool       # Whether to generate visualizations
    
    # Optional parameters with defaults
    sub_sample: int = 1       # Subsampling factor (1=no subsampling)
    samplerIter: int = 0      # Number of sampler iterations
    l_sensitivity: float = 0  # Length-scale sensitivity factor
    estimate_l: bool = False  # Whether to auto-estimate length-scale
    length_scale: float = 1.0 # Manual length-scale (if estimate_l=False)



from sbamp.ds_opt_py.sbamp_header import demo

from sbamp.sed import learn_seds, query_velocity


class SBAMPNode(Node):
    def __init__(self):
        super().__init__("sbamp_node")
        self.get_logger().info("Python sbamp_node has been started.")

        self.declare_parameter('rrt_path_topic', '/rrt_path')

        rrt_path_topic = self.get_parameter('rrt_path_topic').get_parameter_value().string_value


        qos_profile = QoSProfile(depth=10)

        self.rrt_path_subscriber_ = self.create_publisher(Path, rrt_path_topic, self.rrt_path_callback, qos_profile)

    def rrt_path_callback(self, msg):
        self.get_logger().info(f"Received RRT path message: {msg}")

        # # Extract positions and velocities from the Path message
        # positions = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        # velocities = np.array([[pose.pose.orientation.x, pose.pose.orientation.y] for pose in msg.poses])

        # # Learn SEDS parameters
        # gmm, stable_A_matrices, target = learn_seds(positions, velocities)

        # # Query velocity for a specific position
        # query_pos = np.array([0.5, 0.5])
        # velocity = query_velocity(gmm, stable_A_matrices, query_pos)
        # self.get_logger().info(f"Queried velocity: {velocity}")


def main(args=None):
    rclpy.init(args=args)
    node = SBAMPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()