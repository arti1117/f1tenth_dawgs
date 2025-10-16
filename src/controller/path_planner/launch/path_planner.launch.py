#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_name = 'path_planner'
    param_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'planner_params.yaml'
    )

    # Path Planner Node with integrated CSV reading
    path_planner_node = Node(
        package='path_planner',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[param_file]
        # , remappings=[
        #     ('/odom', '/odometry/filtered'),
        # ]
        , remappings=[
            ('/odom', '/ego_racecar/odom'),
        ]
    )

    return LaunchDescription([
        path_planner_node
    ])