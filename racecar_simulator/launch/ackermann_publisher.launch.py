# ackermann_publisher_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racecar_simulator',  # Replace with your actual package name
            executable='ackermann_publisher',
            name='ackermann_publisher',
            output='screen',
        ),
    ])