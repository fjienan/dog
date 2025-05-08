#!/usr/bin/env python3
"""
启动 ares_control_node
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ares_comm',
            executable='ares_control_node',
            name='ares_control_node',
            output='screen'
        )
    ])