from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sbamp",
            executable="sbamp_node.py",
            name="sbamp_node",
            output="screen",
        ),
        Node(
            package="sbamp",
            executable="visualization_node.py",
            name="visualization_node",
            output="screen",
        )
    ])