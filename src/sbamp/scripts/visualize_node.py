#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

from transforms3d.euler import quat2euler

import csv
import numpy as np

from ament_index_python.packages import get_package_share_directory

from scipy.interpolate import CubicSpline

from sbamp.utils import load_waypoints

from sbamp.sed import learn_seds, query_velocity

class VisualizeNode(Node):
    def __init__(self):
        super().__init__("visualize_node")
        self.get_logger().info("Visualize Node Launched")

        self.declare_parameter('waypoint_file_name', 'waypoints_manual_spline.csv')
        self.declare_parameter('visualize_wp_topic', '/visualization/waypoints')
        self.declare_parameter('rrt_path_topic', '/rrt_path')
        self.declare_parameter('visualize_rrt_path_topic', '/visualization/rrt_path')
        self.declare_parameter('visualize_sed_path_topic', '/visualization/sed_path')

        waypoint_file_name = self.get_parameter('waypoint_file_name').get_parameter_value().string_value
        visualize_wp_topic = self.get_parameter('visualize_wp_topic').get_parameter_value().string_value
        rrt_path_topic = self.get_parameter('rrt_path_topic').get_parameter_value().string_value        
        visualize_rrt_path_topic = self.get_parameter('visualize_rrt_path_topic').get_parameter_value().string_value
        visualize_sed_path_topic = self.get_parameter('visualize_sed_path_topic').get_parameter_value().string_value

        package_share_dir = get_package_share_directory("sbamp")

        waypoint_file_path = os.path.join(package_share_dir, 'config', waypoint_file_name)
        self.waypoints = np.array(load_waypoints(waypoint_file_path))

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )   

        # Subscribers
        self.rrt_path_subscriber_ = self.create_subscription(Path, rrt_path_topic, self.visualize_rrt_path, qos_profile)

        self.sed_path_subscriber_ = self.create_subscription(Path, rrt_path_topic, self.visualize_sed_path, qos_profile)

        # Publishers
        self.waypoint_marker_publisher_ = self.create_publisher(MarkerArray, visualize_wp_topic, qos_profile)
        self.rrt_path_marker_publisher_ = self.create_publisher(MarkerArray, visualize_rrt_path_topic, qos_profile)
        self.sed_path_marker_publisher_ = self.create_publisher(MarkerArray, visualize_sed_path_topic, qos_profile)
        
        # self.visualization_timer = self.create_timer(1, self.visualize_waypoints)

        # Visualize the waypoints
        self.visualize_waypoints()

        self.prev_valid_index = 0
        self.prev_ego_waypoints = []
    
    def visualize_waypoints(self):
        marker_array = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            x, y, yaw, qw, qx, qy, qz = wp

            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = Pose()
            marker.pose.position = Point(x=x, y=y, z=0.0)
            marker.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)

            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        self.waypoint_marker_publisher_.publish(marker_array)
        self.get_logger().info("Waypoints Visualized")

    def visualize_rrt_path(self, msg):
        # self.get_logger().info("RRT Path Received")

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 4200
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale = Vector3(x=0.1, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        for i, pose in enumerate(msg.poses):
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.1
            marker.points.append(p)
        
        arrow_marker = Marker()
        arrow_marker.header = marker.header
        arrow_marker.id = 4201
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.scale = Vector3(x=0.3, y=0.15, z=0.1)
        arrow_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        if len(msg.poses) > 0:

            p1 = msg.poses[-2].pose.position
            p2 = msg.poses[-1].pose.position

            arrow_marker.pose.position.x = p2.x
            arrow_marker.pose.position.y = p2.y
            arrow_marker.pose.position.z = 0.2

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            angle = np.arctan2(dy, dx)

            arrow_marker.pose.orientation.z = np.sin(angle / 2.0)
            arrow_marker.pose.orientation.w = np.cos(angle / 2.0)

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        if len(msg.poses) > 2:
            marker_array.markers.append(arrow_marker)
        
        self.rrt_path_marker_publisher_.publish(marker_array)
        # self.get_logger().info("RRT Path Visualized")

    def visualize_sed_path(self, msg):
        rrt_sed_start = np.array([msg.poses[0].pose.position.x, msg.poses[0].pose.position.y])
        rrt_sed_end = np.array([msg.poses[1].pose.position.x, msg.poses[1].pose.position.y])

        # Learn SEDS from the RRT path
        gmm, stable_A, target = learn_seds(rrt_sed_start, rrt_sed_end)
        
        # Create grid for vector field visualization
        x_min, x_max = min(rrt_sed_start[0], rrt_sed_end[0]) - 1.0, max(rrt_sed_start[0], rrt_sed_end[0]) + 1.0
        y_min, y_max = min(rrt_sed_start[1], rrt_sed_end[1]) - 1.0, max(rrt_sed_start[1], rrt_sed_end[1]) + 1.0

        x = np.linspace(x_min, x_max, 5)
        y = np.linspace(y_min, y_max, 5)
        X, Y = np.meshgrid(x, y)
        grid_points = np.column_stack([X.ravel(), Y.ravel()])

        # Query velocities at grid points
        velocities = np.array([query_velocity(gmm, stable_A, target, p) for p in grid_points])

        # Create marker array for vector field
        marker_array = MarkerArray()

        # Vector Field Arrows
        for i, (point, velocity) in enumerate(zip(grid_points, velocities)):
            arrow = Marker()
            arrow.header = Header()
            arrow.header.frame_id = "map"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.id = i + 8400
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            arrow.pose.position = Point(x=point[0], y=point[1], z=0.1)

            # Calculate orientation from velocity vector
            angle = np.arctan2(velocity[1], velocity[0])
            arrow.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=np.sin(angle / 2.0),
                w=np.cos(angle / 2.0)
            )

            # Scale and color
            vel_mag = np.linalg.norm(velocity)
            scale = 0.5
            arrow.scale = Vector3(x=scale*vel_mag, y=0.08, z=0.08)

            dist_to_end = np.linalg.norm(point - target)
            max_dist = np.linalg.norm(rrt_sed_start - target)
            norm_dist = min(dist_to_end / max_dist, 1.0) if max_dist > 0 else 0

            arrow.color = ColorRGBA(r=norm_dist, g=1.0-norm_dist, b=0.2, a=0.7)
            
            arrow.lifetime.sec = 0

            marker_array.markers.append(arrow)

        # Add trajectory line
        traj_marker = Marker()
        traj_marker.header = Header()
        traj_marker.header.frame_id = "map"
        traj_marker.header.stamp = self.get_clock().now().to_msg()
        traj_marker.id = len(grid_points)
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD
        
        # Generate trajectory points
        current_pos = rrt_sed_start.copy()
        traj_points = [Point(x=current_pos[0], y=current_pos[1], z=0.15)]
        
        # Simulate the dynamical system for a few steps
        for _ in range(50):
            velocity = query_velocity(gmm, stable_A, target, current_pos)
            # Scale velocity for smoother visualization
            step_size = 0.05
            current_pos = current_pos - step_size * velocity  # Move toward goal
            traj_points.append(Point(x=current_pos[0], y=current_pos[1], z=0.15))
            
            # Stop if close to goal
            if np.linalg.norm(current_pos - target) < 0.1:
                break
        
        traj_marker.points = traj_points
        traj_marker.scale = Vector3(x=0.05, y=0.0, z=0.0)
        traj_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
        traj_marker.lifetime.sec = 0
        
        marker_array.markers.append(traj_marker)
        
        # Add start and goal markers
        start_marker = Marker()
        start_marker.header = Header()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = len(grid_points) + 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position = Point(x=rrt_sed_start[0], y=rrt_sed_start[1], z=0.2)
        start_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
        start_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        start_marker.lifetime.sec = 0
        marker_array.markers.append(start_marker)
        
        goal_marker = Marker()
        goal_marker.header = Header()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.id = len(grid_points) + 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = Point(x=rrt_sed_end[0], y=rrt_sed_end[1], z=0.2)
        goal_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
        goal_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        goal_marker.lifetime.sec = 0
        
        marker_array.markers.append(start_marker)
        marker_array.markers.append(goal_marker)

        self.sed_path_marker_publisher_.publish(marker_array)
        # self.get_logger().info("SED Path Visualized")
     

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()