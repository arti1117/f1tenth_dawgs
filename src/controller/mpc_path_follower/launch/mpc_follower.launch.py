#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mpc_path_follower')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'mpc_params.yaml')

    # MPC Path Follower Node
    mpc_node = Node(
        package='mpc_path_follower',
        executable='mpc_path_follower_node',
        name='mpc_path_follower',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odom', '/odom'),
            ('/drive', '/drive'),
            ('/frenet_path', '/frenet_path'),
        ]
    )

    return LaunchDescription([
        mpc_node
    ])
