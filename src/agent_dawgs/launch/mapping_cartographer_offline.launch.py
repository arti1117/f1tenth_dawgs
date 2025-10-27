#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    bag_file = LaunchConfiguration('bag_file')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    play_rate = LaunchConfiguration('play_rate', default='1.0')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    cartographer_config_dir = os.path.join(agent_dawgs_prefix, 'config')
    configuration_basename = 'cartographer_mapping_2d.lua'

    rviz_config_dir = os.path.join(agent_dawgs_prefix, 'rviz', 'cartographer_dawgs.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'bag_file',
            description='Full path to the ROS2 bag file for offline mapping'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

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

        # Cartographer node with use_sim_time enabled
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
        ),

        # Occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'resolution': resolution},
                {'publish_period_sec': publish_period_sec}
            ],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
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
