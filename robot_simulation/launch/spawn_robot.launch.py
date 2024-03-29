import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    world = os.path.join(get_package_share_directory('robot_simulation'), 'worlds', 'empty.world')
    urdf = os.path.join(get_package_share_directory('robot_simulation'), 'urdf', 'hoverboard.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'),

        ExecuteProcess(
           cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity',
                'hoverboard', '-file', urdf, '-x 0.0', '-y 0.0'],
           output='screen'),
    ])
