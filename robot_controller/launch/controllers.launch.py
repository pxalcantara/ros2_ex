#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    move_controller_node = Node(
        package='robot_controller',
        executable='move_controller_node'
    )

    path_controller_node = Node(
        package='robot_controller',
        executable='path_controller_node.py'
    )

    return LaunchDescription([
      path_controller_node,
      move_controller_node
    ])
