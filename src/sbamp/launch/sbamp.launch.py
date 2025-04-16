from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Python node
        Node(
            package="sbamp",
            executable="sbamp_node.py",
            name="sbamp_node",
            output="screen",
        ),
        # # C++ node
        # Node(
        #     package="sbamp",
        #     executable="sbamp_node",
        #     name="sbamp_node",
        #     output="screen",
        # )
        # RRT node
        Node(
            package="sbamp",
            executable="rrt_node.py",
            name="rrt_node",
            output="screen",
        ),
    ])