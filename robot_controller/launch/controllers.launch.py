#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
