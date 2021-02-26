import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    world = os.path.join(get_package_share_directory('hoverboard_mvp'), 'worlds', 'empty.world')
    urdf = os.path.join(get_package_share_directory('hoverboard_mvp'), 'urdf', 'hoverboard.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'),
            
        ExecuteProcess(
           cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity',
                'squarbo', '-file', urdf, '-x 0.5', '-y 2.05' ],
           output='screen'),

    ])
