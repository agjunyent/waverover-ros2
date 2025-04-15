#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waverover_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('waverover_controller'),
                'launch',
                'waverover_controller.launch.py'
            )
        ),
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        ),
    )

    cameras_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cameras_controller'),
                'launch',
                'cameras_controller_launch.py'
            )
        ),
    )

    waverover_description_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('waverover_description'),
                'launch',
                'description.launch.xml'
            )
        ),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={'params_file': os.path.join(get_package_share_directory('waverover_bringup'), 'config', 'slam_toolbox_online_async.yaml')
                          }.items()
    )

    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('waverover_bringup'), 'config', 'ekf.yaml')],
            remappings=[
            ]
        )

    return LaunchDescription([
        # waverover_controller_launch,
        # sllidar_launch,
        cameras_controller_launch
        # waverover_description_launch,
        # slam_toolbox_launch,
        # robot_localization_node
    ])