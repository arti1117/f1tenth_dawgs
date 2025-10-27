#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    bag_file = LaunchConfiguration('bag_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    play_rate = LaunchConfiguration('play_rate', default='1.0')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    rviz_config_dir = os.path.join(agent_dawgs_prefix, 'rviz', 'cartographer_dawgs.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'bag_file',
            description='Full path to the ROS2 bag file for offline mapping'),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("agent_dawgs"),
                'config',
                'mapper_params_online_async_mapping.yaml'
            ]),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'),

        DeclareLaunchArgument(
            'play_rate',
            default_value='1.0',
            description='Playback rate for the bag file (1.0 = real-time)'),

        # Play bag file with clock publishing
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file,
                 '--clock', '--rate', play_rate],
            output='screen',
            name='bag_player'
        ),

        # SLAM Toolbox node with use_sim_time enabled
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': True}],
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
