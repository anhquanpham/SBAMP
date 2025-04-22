#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

import numpy as np
from transforms3d.euler import quat2euler
from sbamp.utils import load_waypoints, get_grid_coordinates, update_grid_with_ray

# Define a custom Node class for RRT* tree (this is separate from ROS Node)
class RRTTreeNode:
    def __init__(self, position, parent=None, cost=0):
        """
        RRT Tree Node to represent positions in the tree.
        :param position: The (x, y) coordinates of the node
        :param parent: The parent node (used for path extraction)
        :param cost: The cost to reach this node (sum of previous path costs)
        """
        self.position = position  # Position (x, y)
        self.parent = parent  # Parent node (used for backtracking the path)
        self.cost = cost  # Path cost to reach this node from the start

class RRTNode(Node):
    def __init__(self):
        super().__init__("rrt_node")
        self.get_logger().info("Python rrt_node has been started.")

        self.declare_parameter('pose_topic', '/ego_racecar/odom')
        self.declare_parameter('occupancy_grid_topic', '/occupancy_grid')
        self.declare_parameter('next_wp_topic', '/next_waypoint')
        self.declare_parameter('rrt_path_topic', '/rrt_path')

        self.declare_parameter('map_height', 713)
        self.declare_parameter('map_width', 727)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_origin', (-20.2, -5.68))

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').get_parameter_value().string_value
        next_wp_topic = self.get_parameter('next_wp_topic').get_parameter_value().string_value
        rrt_path_topic = self.get_parameter('rrt_path_topic').get_parameter_value().string_value

        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_origin = self.get_parameter('map_origin').get_parameter_value().double_array_value
        
        occupancy_grid_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )  

        # NOTE: self.occupancy_grid is defined and set inside occupancy_grid_callback
        self.occupancy_grid_subscriber_ = self.create_subscription(OccupancyGrid, occupancy_grid_topic, self.occupancy_grid_callback, occupancy_grid_qos_profile)


        qos_profile = QoSProfile(depth=10)

        # NOTE: 
        self.next_waypoint_subscriber_ = self.create_subscription(PointStamped, next_wp_topic, self.next_waypoint_callback, qos_profile)

        # NOTE: self.cur_pos and self.cur_yaw is defined and set inside pose_callback
        self.pose_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, qos_profile)


        self.path_publisher_ = self.create_publisher(Path, rrt_path_topic, qos_profile)

        self.step_size = 1.0  # Define the step size for steering




    def occupancy_grid_callback(self, occ_grid_msg):
        # Process the OccupancyGrid message
        self.map_height = occ_grid_msg.info.height
        self.map_width = occ_grid_msg.info.width
        self.map_resolution = occ_grid_msg.info.resolution
        self.map_origin = (occ_grid_msg.info.origin.position.x, occ_grid_msg.info.origin.position.y)

        self.occupancy_grid = np.array(occ_grid_msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))

        
        # self.get_logger().info(f"Received map and occupancy_grid set: \n"
        #                        f"height: {self.map_height}, width: {self.map_width},\n"
        #                        f"resolution: {self.map_resolution}, \n"
        #                        f"origin: {self.map_origin}"
        #                        )

    def next_waypoint_callback(self, waypoint_msg):
        # Process the PointStamped message
        self.next_waypoint = np.array([waypoint_msg.point.x, waypoint_msg.point.y])
        
        # self.get_logger().info(f"Next waypoint received: {self.next_waypoint}")
        

    def pose_callback(self, pose_msg):
        # Process the Odometry message
        self.cur_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        self.cur_yaw = quat2euler([pose_msg.pose.pose.orientation.w,
                                          pose_msg.pose.pose.orientation.x,
                                          pose_msg.pose.pose.orientation.y,
                                          pose_msg.pose.pose.orientation.z])[2]

        if not hasattr(self, 'occupancy_grid'):
            self.get_logger().error("Occupancy grid not received yet.")
            return
        
        if not hasattr(self, 'next_waypoint'):
            self.get_logger().error("Next waypoint not received yet.")
            return

        # self.get_logger().info(f"Current position: {self.cur_pos}, Current yaw: {self.cur_yaw}, Next waypoint: {self.next_waypoint}")
        
        # current position: self.cur_pos
        # current yaw: self.cur_yaw
        # occupancy grid: self.occupancy_grid
        # next waypoint: self.next_waypoint

        # Implement RRT algorithm here

        # Call the RRT* algorithm


        ############################### ADD ##############################################################

        goal = self.next_waypoint
        path = self.rrt_star(self.cur_pos, goal)

        if path:
            # self.get_logger().info(f"Path found: {len(path)}")

            # Publish the path as a Path message
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for point in path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.path_publisher_.publish(path_msg)
            
        # else:
        #     self.get_logger().error("Failed to find path!")

    def rrt_star(self, start, goal, max_iter=10, step_size=1.0, goal_threshold=0.5):
        # Initialize the tree with the start node
        tree = [RRTTreeNode(start)]
        self.goal_reached = False

        for i in range(max_iter):
            # Sample a random point
            random_point = self.sample_random_point(goal)

            # self.get_logger().info(f"Random point sampled: {random_point}")

            # Find the nearest node in the tree
            nearest_node = self.nearest_node(tree, random_point)

            # self.get_logger().info(f"Nearest node found: {nearest_node.position}")

            # Steer towards the random point from nearest node
            new_node = self.steer(nearest_node, random_point, step_size)

            # self.get_logger().info(f"New node created: {new_node.position}")

            # Check if the new node is valid (i.e., no collision)
            if not self.is_valid(new_node.position):
                continue

            # self.get_logger().info(f"New node is valid: {new_node.position}")

            # # Rewire the tree to optimize the path

            old_tree = tree.copy()

            self.rewire_tree(tree, new_node, step_size)

            # if old_tree == tree:
            #     self.get_logger().info(f"New node is not rewired: {new_node.position}")
            # else:
            #     self.get_logger().info(f"New node rewired: {new_node.position}")

            # Check if the goal is reached
            if np.linalg.norm(new_node.position - goal) < goal_threshold:
                self.goal_reached = True
                return self.extract_path(new_node)

            tree.append(new_node)

        # self.get_logger().info("Goal wasnt reached") 
            
        return None  # Return None if the goal is not reached

    def sample_random_point(self, goal, bias_factor=0.9):
        """
        Sample a random point in the space with a bias towards the goal.
        `goal` is the next waypoint (the goal position).
        `bias_factor` determines how biased the sampling is towards the goal.
        """
        # Randomly decide if we want to sample towards the goal or randomly
        if np.random.rand() < bias_factor:
            # Sample towards the goal (more likely to sample towards the goal)
            direction = goal - self.cur_pos
            distance = np.linalg.norm(direction)
            
            # Generate a random point along the line to the goal, within step_size
            step_size = self.step_size 
            if distance < step_size:
                return goal  # If we're close enough to the goal, return the goal
            
            # Normalize direction and move a step towards the goal
            direction /= distance  # Normalize the direction
            random_point = self.cur_pos + direction * step_size
        else:
            # Uniformly sample from the entire map
            random_point = np.array([
                np.random.uniform(self.map_origin[0], self.map_origin[0] + self.map_width * self.map_resolution),
                np.random.uniform(self.map_origin[1], self.map_origin[1] + self.map_height * self.map_resolution)
            ])

        return random_point

    def nearest_node(self, tree, point):
        # Find the nearest node in the tree to the random point
        distances = [np.linalg.norm(node.position - point) for node in tree]
        return tree[np.argmin(distances)]

    def steer(self, nearest_node, random_point, step_size):
        # Move from nearest node towards the random point by step_size
        direction = random_point - nearest_node.position
        distance = np.linalg.norm(direction)
        
        if distance < step_size:
            return RRTTreeNode(random_point, nearest_node)
        
        direction /= distance  # Normalize direction
        new_position = nearest_node.position + direction * step_size
        return RRTTreeNode(new_position, nearest_node)

    def is_valid(self, position):
        # Check if the position is valid (not inside an obstacle)
        x, y = position
        grid_x, grid_y = get_grid_coordinates(x, y, self.map_origin, self.map_resolution)
        
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            return self.occupancy_grid[grid_y, grid_x] == 0  # 0 is free space
        return False

    def rewire_tree(self, tree, new_node, step_size):
        # Try to rewire the tree by checking if the new node offers a better path
        for node in tree:
            # Check if the new node can reach any nearby node with a lower cost
            if np.linalg.norm(new_node.position - node.position) < step_size:
                new_cost = new_node.cost + np.linalg.norm(new_node.position - node.position)
                if new_cost < node.cost:
                    node.parent = new_node
                    node.cost = new_cost

    def extract_path(self, goal_node):
        # Backtrack to extract the path
        path = []
        current_node = goal_node
        while current_node is not None:
            path.append(current_node.position)
            current_node = current_node.parent
        return path[::-1]  # Reverse the path to get it from start to goal

    ############################### ADD ##############################################################


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()