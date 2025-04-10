#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion
from geometry_msgs.msg import Vector3
import csv
import numpy as np

class VisualizationNode(Node):
    def __init__(self):
        super().__init__("visualization_node")
        self.get_logger().info("Python visualization_node has been started.")

        # Configuration
        waypoint_file_name = "/config/levine_waypoints.csv"
        visualization_topic = "/visualization/waypoints"

        # Load Waypoints from CSV file
        package_share_dir = get_package_share_directory("sbamp")
        absolute_file_path = package_share_dir + waypoint_file_name
        self.waypoints = []

        try:        
            with open(absolute_file_path, 'r') as csvfile:
                for row in csv.reader(csvfile):
                    self.waypoints.append((float(row[0]), float(row[1])))
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {absolute_file_path}")
            return
        
        self.waypoints = np.array(self.waypoints)
        
        qos_profile = QoSProfile(depth=10)

        self.waypoint_marker_publisher = self.create_publisher(MarkerArray, visualization_topic, qos_profile)

        self.publish_waypoint_markers()

        # Publish the waypoints as markers
        # self.visualization_timer = self.create_timer(5.0, self.publish_waypoint_markers)
    
    def publish_waypoint_markers(self):
        marker_array = MarkerArray()

        if self.waypoints is not None:
            for waypoint_index, waypoint in enumerate(self.waypoints):
                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = waypoint_index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose = Pose()
                marker.pose.position.x = waypoint[0]
                marker.pose.position.y = waypoint[1]
                marker.pose.position.z = 0.0
                marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

                marker.scale = Vector3(x=0.1, y=0.1, z=0.1)

                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

                marker.lifetime.sec = 0

                marker_array.markers.append(marker)
            
            self.waypoint_marker_publisher.publish(marker_array)
    
def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()