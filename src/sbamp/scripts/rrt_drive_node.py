#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
    
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
from transforms3d.euler import quat2euler

from sbamp.sed import learn_seds, query_velocity

class RRTDriveNode(Node):
    def __init__(self):
        super().__init__("rrt_drive_node")
        self.get_logger().info("Python rrt_drive_node has been started.")

        self.declare_parameter('rrt_path_topic', '/rrt_path')
        self.declare_parameter('pose_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('kv', 1.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('wheelbase', 0.325)

        rrt_path_topic = self.get_parameter('rrt_path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.kv = self.get_parameter('kv').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value

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

    def pose_callback(self, pose_msg):
        self.cur_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        self.cur_yaw = quat2euler([pose_msg.pose.pose.orientation.w,
                                        pose_msg.pose.pose.orientation.x,
                                        pose_msg.pose.pose.orientation.y,
                                        pose_msg.pose.pose.orientation.z])[2]
        
        if not hasattr(self, 'rrt_waypoints'):
            self.get_logger().error("RRT path not received yet.", throttle_duration_sec=1.0)
            return
        
        # Find the next waypoint
        next_wp = self.find_rrt_lookup_waypoint(self.cur_pos[0], self.cur_pos[1], self.cur_yaw)

        if next_wp is None:
            self.get_logger().error("No valid waypoint found.", throttle_duration_sec=1.0)
            return
        
        # Calculate the curvature
        x_ego = next_wp[0]
        y_ego = next_wp[1]

        curvature = 2 * y_ego * self.wheelbase / (x_ego**2 + y_ego**2)

        self.publish_drive_command(curvature)

    def find_rrt_lookup_waypoint(self, pos_x, pos_y, yaw):
        # Find the closest waypoint based on the current position and yaw
        closest_wp = None
        closest_distance = float("inf")

        for wp in self.rrt_waypoints[1:]:
            wp_x, wp_y = wp
            dx = wp_x - pos_x
            dy = wp_y - pos_y
            distance = np.sqrt(dx**2 + dy**2)

            x_ego = dx * np.cos(-yaw) - dy * np.sin(-yaw)
            y_ego = dx * np.sin(-yaw) + dy * np.cos(-yaw)

            if distance < closest_distance and x_ego >= 0:
                closest_distance = distance
                closest_wp = [x_ego, y_ego]

        return closest_wp

    def publish_drive_command(self, curvature):

        steering_angle = self.kp * curvature

        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        speed_multiplier = 1.0 - self.kv * (abs(steering_angle) / self.max_steering_angle)
        speed = self.max_speed * speed_multiplier

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher_.publish(drive_msg)
        self.get_logger().info(f"Speed={speed}, Steering Angle={steering_angle}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RRTDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()