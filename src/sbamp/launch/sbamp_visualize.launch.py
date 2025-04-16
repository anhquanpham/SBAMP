from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SBAMP node
        Node(
            package="sbamp",
            executable="sbamp_node.py",
            name="sbamp_node",
            output="screen",
        ),
        # Visualize node
        Node(
            package="sbamp",
            executable="visualize_node.py",
            name="visualize_node",
            output="screen",
            parameters=[
                {"waypoint_file_name": "waypoints_levine.csv"},
                {"pose_topic": "/ego_racecar/odom"},
                {"visualize_wp_topic": "/visualization/waypoints"},
                {"next_wp_topic": "/visualization/next_waypoint"},
                {"lookahead_distance": 0.8},
                {"y_ego_threshold": 1.2}
            ]
        ),
        # RRT node
        Node(
            package="sbamp",
            executable="rrt_node.py",
            name="rrt_node",
            output="screen",
        ),
    ])