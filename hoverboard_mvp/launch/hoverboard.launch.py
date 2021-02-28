#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    world_file = 'empty.world'
    urdf_file_name = 'hoverboard.urdf'
    xacro_file_name = 'hoverboard.urdf.xacro'
    world_path = os.path.join(get_package_share_directory('hoverboard_mvp'), 'worlds', world_file)
    urdf_path = os.path.join(get_package_share_directory('hoverboard_mvp'), 'urdf')
    urdf_file = os.path.join(get_package_share_directory('hoverboard_mvp'), 'urdf', urdf_file_name)
    xacro_file = os.path.join(get_package_share_directory('hoverboard_mvp'), 'urdf', xacro_file_name)
    # urdf_file = os.path.join('/root/home/foxy_ws/src/tech_palcantara/hoverboard_mvp/', 'urdf', 'squarbo.urdf')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    gz_launch = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
    )

    # teleop node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard'
        # remappings=[
        #     ('cmd_vel', '/demo/cmd_demo')
        # ]
    ) 

    # doc = xacro.process_file(xacro_file)
    # robot_desc = doc.toprettyxml(indent='  ')

    # print ('======================================================================================')
    # print (robot_desc)
    # print ('======================================================================================')
    
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'squarbo', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-file', urdf_file],
        output='screen'
    )

    move_controller_node = Node(
        package='hoverboard_mvp',
        executable='move_controller_node'
    )

    return LaunchDescription([
        gz_launch,
        # move_controller_node
        spawn

       
    ])