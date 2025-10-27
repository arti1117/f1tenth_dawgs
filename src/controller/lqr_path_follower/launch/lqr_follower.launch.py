#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lqr_path_follower')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'lqr_params.yaml')

    # LQR Path Follower Node
    lqr_node = Node(
        package='lqr_path_follower',
        executable='lqr_path_follower_node',
        name='lqr_path_follower',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odom', '/ego_racecar/odom'),
            # ('/odom', '/pf/'),
            ('/drive', '/drive'),
            ('/frenet_path', '/frenet_path'),
        ]
    )

    return LaunchDescription([
        lqr_node
    ])
