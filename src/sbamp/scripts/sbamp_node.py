#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sbamp.ds_opt_py.lpv_opt.optimize_lpv_ds_from_data import optimize_lpv_ds_from_data
from sbamp.ds_opt_py.lpv_opt.lpv_ds import lpv_ds
    
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped

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
        self.declare_parameter('drive_topic', '/drive')


        rrt_path_topic = self.get_parameter('rrt_path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        qos_profile = QoSProfile(depth=10)

        # NOTE: self.rrt_waypoints is defined and set inside rrt_path_callback
        self.rrt_path_subscriber_ = self.create_subscription(Path, rrt_path_topic, self.rrt_path_callback, qos_profile)
        
        # NOTE: self.cur_pos and self.cur_yaw is defined and set inside pose_callback
        self.pose_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, qos_profile)

        self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, qos_profile)


    def rrt_path_callback(self, msg):
        # self.get_logger().info(f"Received RRT path message: {len(msg.poses)} poses")

        # Extract positions from the Path message
        self.rrt_waypoints = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])       

        # Learn SEDS from the RRT path
        self.gmm, self.stable_A_matrices, self.target = learn_seds(self.rrt_waypoints[0], self.rrt_waypoints[1])
        # self.get_logger().info(f"waypoints 1 and waypoints 2: {self.rrt_waypoints[0]} and {self.rrt_waypoints[1]}") 

    def pose_callback(self, pose_msg):
        self.cur_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        self.cur_yaw = quat2euler([pose_msg.pose.pose.orientation.w,
                                        pose_msg.pose.pose.orientation.x,
                                        pose_msg.pose.pose.orientation.y,
                                        pose_msg.pose.pose.orientation.z])[2]
        
        if not hasattr(self, 'rrt_waypoints'):
            self.get_logger().error("RRT path not received yet.", throttle_duration_sec=1.0)
            return
        
        velocity = query_velocity(self.gmm, self.stable_A_matrices, self.target, self.cur_pos)

        self.publish_drive_command(velocity)

    def publish_drive_command(self, velocity):
        speed = np.linalg.norm(velocity)
        steering_angle = np.arctan2(velocity[1], velocity[0]) - self.cur_yaw

        # Normalize steering angle to [-pi, pi]
        steering_angle = (steering_angle + np.pi) % (2 * np.pi) - np.pi
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher_.publish(drive_msg)
        self.get_logger().info(f"Speed={speed}, Steering Angle={steering_angle}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SBAMPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()