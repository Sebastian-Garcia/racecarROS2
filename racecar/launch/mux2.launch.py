# -*- mode: Python -*-
# This work is sponsored by the Department of the Air Force under Air Force
# Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
# recommendations are those of the author and are not necessarily endorsed by
# the United States Government.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


#TODO: remove arguments for each, and hardcode them directly in each specific relay file:

def generate_launch_description():
    return LaunchDescription([
        # Chain the MUXs
        Node(
            package='racecar',
            executable='chainRelay',
            name='mux_chainer',
            output='screen',
            # arguments=[
            #     '/vesc/high_level/ackermann_cmd_mux/output',
            #     '/vesc/low_level/ackermann_cmd_mux/input/navigation'
            # ]
        ),

        # Define mappings for backwards compatibility
        Node(
            package='racecar',
            executable='safetyRelay',
            name='mux_topic_backward_compat_safety',
            output='screen',
            # arguments=[
            #     '/vesc/ackermann_cmd_mux/input/safety',
            #     '/vesc/low_level/ackermann_cmd_mux/input/safety'
            # ]
        ),
        Node(
            package='racecar',
            executable='teleopRelay',
            name='mux_topic_backward_compat_teleop',
            output='screen',
            # arguments=[
            #     '/vesc/ackermann_cmd_mux/input/teleop',
            #     '/vesc/low_level/ackermann_cmd_mux/input/teleop'
            # ]
        ),
        Node(
            package='racecar',
            executable='navRelay',
            name='mux_topic_backward_compat_navigation',
            output='screen',
            # arguments=[
            #     '/vesc/ackermann_cmd_mux/input/navigation',
            #     '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
            # ]
        ),

        # # Default (zero) ackermann command for high level MUX
        # Node(
        #     package='ros2topic',
        #     executable='ros2topic',
        #     name='zero_ackermann_cmd',
        #     output='screen',
        #     arguments=[
        #         'pub',
        #         '-r', '6',
        #         '/high_level/ackermann_cmd_mux/input/default',
        #         'ackermann_msgs/msg/AckermannDriveStamped',
        #         '{"header": {"stamp": {"sec": 0, "nanosec": 0}}, "drive": {"steering_angle": 0.0, "speed": 0.0}}'
        #     ]
        # ),
        Node(
            package='racecar',
            executable='defaultCmd',
            name='zero_ackermann_cmd',
            output='screen'
        ),
        
        # High level MUX
        Node(
            package='ackermann_cmd_mux',
            executable='ackermann_cmd_mux_node',
            name='high_level_mux',
            namespace='high_level',
            output='screen',
            parameters=[{'config_file': 'racecar/config/high_level_mux.yaml'}]
        ),

        # Low level MUX
        Node(
            package='ackermann_cmd_mux',
            executable='ackermann_cmd_mux_node',
            name='low_level_mux',
            namespace='low_level',
            output='screen',
            parameters=[{'config_file': 'racecar/config/low_level_mux.yaml'}]
        ),
    ])