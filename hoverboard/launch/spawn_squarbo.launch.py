import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    world = os.path.join(get_package_share_directory('squarbo_gazebo'), 'gazebo_ros_state.world')
    # urdf = os.path.join(get_package_share_directory('squarbo_description'), 'urdf', 'squarbo.urdf')
    urdf = os.path.join(get_package_share_directory('squarbo_description'), 'urdf', 'hoverboard.urdf')
    # rviz = os.path.join(get_package_share_directory('squarbo_gazebo'), 'rviz', 'squarbo_model.rviz')

    # xml = open(urdf, 'r').read()

    # xml = xml.replace('"', '\\"')

    # , robot_namespace: \"/squarbo\"
    # swpan_args = '{name: \"squarbo\", xml: \"'  +  xml + '\"}'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'),

        # ExecuteProcess(
        #     cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
        #     output='screen'),
            
        ExecuteProcess(
           cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity',
                'squarbo', '-file', urdf, '-x 0.5', '-y 0.05', '-Y 0.2'],
           output='screen'),

    ])
