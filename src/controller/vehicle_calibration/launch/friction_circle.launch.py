#!/usr/bin/env python3
"""
Launch file for Friction Circle Test

This launch file starts the friction circle node with configurable parameters
for testing the vehicle's maximum cornering capabilities.

Usage:
    ros2 launch vehicle_calibration friction_circle.launch.py
    ros2 launch vehicle_calibration friction_circle.launch.py max_speed:=5.0 plot_real_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    declare_max_speed = DeclareLaunchArgument(
        'max_speed',
        default_value='3.0',
        description='Maximum test speed (m/s)'
    )

    declare_min_speed = DeclareLaunchArgument(
        'min_speed',
        default_value='0.5',
        description='Minimum test speed (m/s)'
    )

    declare_speed_increment = DeclareLaunchArgument(
        'speed_increment',
        default_value='0.2',
        description='Speed increment per test (m/s)'
    )

    declare_test_duration = DeclareLaunchArgument(
        'test_duration',
        default_value='10.0',
        description='Duration for each speed test (s)'
    )

    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.33',
        description='Vehicle wheelbase (m)'
    )

    declare_max_steering_angle = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.4',
        description='Maximum steering angle (rad)'
    )

    declare_plot_real_time = DeclareLaunchArgument(
        'plot_real_time',
        default_value='true',
        description='Enable real-time plotting'
    )

    declare_save_data = DeclareLaunchArgument(
        'save_data',
        default_value='true',
        description='Save data to file'
    )

    declare_output_directory = DeclareLaunchArgument(
        'output_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/data/friction_circle',
        description='Output directory for data files'
    )

    # Friction Circle Node
    friction_circle_node = Node(
        package='vehicle_calibration',
        executable='friction_circle_node.py',
        name='friction_circle_node',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': LaunchConfiguration('min_speed'),
            'speed_increment': LaunchConfiguration('speed_increment'),
            'test_duration': LaunchConfiguration('test_duration'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'plot_real_time': LaunchConfiguration('plot_real_time'),
            'save_data': LaunchConfiguration('save_data'),
            'output_directory': LaunchConfiguration('output_directory'),
        }],
        remappings=[
            ('/drive', '/drive'),
            ('/odom', '/odom'),
            ('/imu', '/imu'),
        ]
    )

    return LaunchDescription([
        declare_max_speed,
        declare_min_speed,
        declare_speed_increment,
        declare_test_duration,
        declare_wheelbase,
        declare_max_steering_angle,
        declare_plot_real_time,
        declare_save_data,
        declare_output_directory,
        friction_circle_node,
    ])