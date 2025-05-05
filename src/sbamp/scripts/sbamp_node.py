#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sbamp.ds_opt_py.lpv_opt.optimize_lpv_ds_from_data import optimize_lpv_ds_from_data
from sbamp.ds_opt_py.lpv_opt.lpv_ds import lpv_ds
    
from nav_msgs.msg import Path, Odometry

import numpy as np
from transforms3d.euler import quat2euler

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
        self.declare_parameter('pose_topic', '/ego_racecar/odom')

        rrt_path_topic = self.get_parameter('rrt_path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        qos_profile = QoSProfile(depth=10)

        # NOTE: self.cur_pos and self.cur_yaw is defined and set inside pose_callback
        self.pose_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, qos_profile)

        self.rrt_path_subscriber_ = self.create_subscription(Path, rrt_path_topic, self.rrt_path_callback, qos_profile)

    def pose_callback(self, pose_msg):
        self.cur_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        self.cur_yaw = quat2euler([pose_msg.pose.pose.orientation.w,
                                        pose_msg.pose.pose.orientation.x,
                                        pose_msg.pose.pose.orientation.y,
                                        pose_msg.pose.pose.orientation.z])[2]

    def rrt_path_callback(self, msg):
        self.get_logger().info(f"Received RRT path message: {len(msg.poses)} poses")

        # Extract positions from the Path message
        positions = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])

        if not hasattr(self, 'cur_pos'):
            self.get_logger().error("Current position not received yet.")
            return
        if not hasattr(self, 'cur_yaw'):
            self.get_logger().error("Current yaw not received yet.")
            return



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