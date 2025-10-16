#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('path_planner')

    # Declare launch arguments
    csv_file_path_arg = DeclareLaunchArgument(
        'csv_file_path',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/racetracks/levine/levine_blacked_lippboyd_speedopted_V2.csv',
        description='Path to the CSV file containing waypoints (x,y,v,kappa format)'
    )

    global_path_topic_arg = DeclareLaunchArgument(
        'global_path_topic',
        default_value='/global_centerline',
        description='Topic name for publishing the global path'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the published path'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Rate (Hz) at which to publish the path'
    )

    # CSV Path Publisher Node
    csv_path_publisher_node = Node(
        package='path_planner',
        executable='csv_path_publisher',
        name='csv_path_publisher',
        output='screen',
        parameters=[{
            'csv_file_path': LaunchConfiguration('csv_file_path'),
            'global_path_topic': LaunchConfiguration('global_path_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate')
        }]
    )

    return LaunchDescription([
        csv_file_path_arg,
        global_path_topic_arg,
        frame_id_arg,
        publish_rate_arg,
        csv_path_publisher_node
    ])