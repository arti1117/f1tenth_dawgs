#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_bringup = LaunchConfiguration('use_bringup', default='true')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    f1tenth_stack_prefix = get_package_share_directory('f1tenth_stack')

    particle_filter_config = LaunchConfiguration('particle_filter_config',
        default=os.path.join(
            get_package_share_directory('particle_filter'),
            'config',
            'localize.yaml'
        ))

    rviz_config_dir = os.path.join(agent_dawgs_prefix, 'rviz', 'cartographer_dawgs.rviz')

    # F1TENTH bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(f1tenth_stack_prefix, 'launch', 'bringup_launch.py')
        ]),
        condition=IfCondition(use_bringup)
    )

    # Map file configuration
    DeclareLaunchArgument(
        'map_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/icheon'
    ),
    DeclareLaunchArgument(
        'map_name',
        default_value='icheon1009_map'
    ),
    map_directory = LaunchConfiguration('map_directory')
    map_name = LaunchConfiguration('map_name')

    yaml_file = LaunchConfiguration('yaml_file',
        default='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/icheon/icheon1009_map.yaml'),

    return LaunchDescription([
        DeclareLaunchArgument(
            'particle_filter_config',
            default_value=particle_filter_config,
            description='Full path to particle filter config file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'),
        DeclareLaunchArgument(
            'use_bringup',
            default_value='true',
            description='Launch F1TENTH hardware bringup'),

        # F1TENTH hardware bringup
        bringup_launch,

        # Particle filter node
        Node(
            package='particle_filter',
            executable='particle_filter',
            name='particle_filter',
            output='screen',
            parameters=[
                particle_filter_config,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odom', 'odometry/filtered'),
            ],
        ),

        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': yaml_file,
            }],
            output='screen'),

        # Lifecycle manager for map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
