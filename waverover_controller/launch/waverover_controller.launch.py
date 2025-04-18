#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('waverover_controller'),
        'config',
        'params.yaml')

    return LaunchDescription([
        Node(
            package='waverover_controller',
            executable='waverover_controller',
            name='waverover_controller',
            output='screen',
            parameters=[config],
        ),
    ])