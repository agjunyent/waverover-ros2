import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cameras_controller'),
        'config',
        'params.yaml')

    cameras_controller_node = Node(
        package='cameras_controller',
        executable='cameras_controller_node',
        name='cameras_controller',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        cameras_controller_node
    ])
