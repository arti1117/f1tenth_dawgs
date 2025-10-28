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
    use_ekf = LaunchConfiguration('use_ekf', default='true')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    f1tenth_gym_ros_prefix = get_package_share_directory('f1tenth_gym_ros')

    particle_filter_config = LaunchConfiguration('particle_filter_config',
        default=os.path.join(
            get_package_share_directory('agent_dawgs'),
            'config',
            'pf_localize.yaml'
        ))

    rviz_config_dir = os.path.join(agent_dawgs_prefix, 'rviz', 'particle_localization.rviz')

    # F1TENTH bringup launch (without static map->odom for localization)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(f1tenth_gym_ros_prefix, 'launch', 'gym_bridge_launch.py')
        ]),
        launch_arguments={'publish_map_odom_tf': 'false'}.items(),
        condition=IfCondition(use_bringup)
    )

    # Map file configuration
    # Edit map file in f1tenth_gym_ros/config/sim_path

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
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Launch EKF for odometry filtering'),

        # F1TENTH hardware bringup
        bringup_launch,

        # Particle filter node - started after map server through respawn delay
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
                ('odom', '/ego_racecar/odom'),
            ],
            # Add respawn to handle startup race condition
            respawn=True,
            respawn_delay=3.0,
        ),
    ])
