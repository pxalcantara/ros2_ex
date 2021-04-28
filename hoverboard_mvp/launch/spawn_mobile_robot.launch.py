import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    world = os.path.join(get_package_share_directory('hoverboard_mvp'), 'worlds', 'empty.world')
    # urdf = os.path.join(get_package_share_directory('hoverboard_mvp'), 'urdf', 'hoverboard.urdf')
    urdf_file = os.path.join(get_package_share_directory('mobile_robot_description'), 'urdf/quimera_robot.urdf.xacro')
    # doc = xacro.parse(open(urdf_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    robot = xacro.process(urdf_file)
    params = {'robot_description': robot}

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                        '-entity', 'cartpole'],
            output='screen')

    ])
