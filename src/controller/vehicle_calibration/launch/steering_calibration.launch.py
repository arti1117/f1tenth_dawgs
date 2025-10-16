#!/usr/bin/env python3
"""
Launch file for Steering Gain Calibration

This launch file starts the steering calibration node to determine the optimal
relationship between commanded steering angle and actual wheel angle.

Usage:
    ros2 launch vehicle_calibration steering_calibration.launch.py
    ros2 launch vehicle_calibration steering_calibration.launch.py test_speed:=1.5 max_steering_angle:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    declare_min_steering_angle = DeclareLaunchArgument(
        'min_steering_angle',
        default_value='-0.4',
        description='Minimum steering angle (rad)'
    )

    declare_max_steering_angle = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.4',
        description='Maximum steering angle (rad)'
    )

    declare_steering_increment = DeclareLaunchArgument(
        'steering_increment',
        default_value='0.05',
        description='Steering increment per test (rad)'
    )

    declare_test_speed = DeclareLaunchArgument(
        'test_speed',
        default_value='1.0',
        description='Constant test speed (m/s)'
    )

    declare_test_duration = DeclareLaunchArgument(
        'test_duration',
        default_value='12.0',
        description='Duration for each steering test (s)'
    )

    declare_settling_time = DeclareLaunchArgument(
        'settling_time',
        default_value='2.0',
        description='Time to let steering settle (s)'
    )

    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.33',
        description='Vehicle wheelbase (m)'
    )

    declare_min_turn_radius = DeclareLaunchArgument(
        'min_turn_radius',
        default_value='0.5',
        description='Minimum valid turning radius (m)'
    )

    declare_max_turn_radius = DeclareLaunchArgument(
        'max_turn_radius',
        default_value='20.0',
        description='Maximum valid turning radius (m)'
    )

    declare_initial_steering_gain = DeclareLaunchArgument(
        'initial_steering_gain',
        default_value='1.0',
        description='Initial steering gain estimate'
    )

    declare_initial_steering_offset = DeclareLaunchArgument(
        'initial_steering_offset',
        default_value='0.0',
        description='Initial steering offset (rad)'
    )

    declare_plot_results = DeclareLaunchArgument(
        'plot_results',
        default_value='true',
        description='Enable result plotting'
    )

    declare_save_calibration = DeclareLaunchArgument(
        'save_calibration',
        default_value='true',
        description='Save calibration data'
    )

    declare_output_directory = DeclareLaunchArgument(
        'output_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/data/calibration',
        description='Output directory for calibration files'
    )

    declare_use_least_squares_circle = DeclareLaunchArgument(
        'use_least_squares_circle',
        default_value='true',
        description='Use least squares circle fitting'
    )

    # Steering Calibration Node
    steering_calibration_node = Node(
        package='vehicle_calibration',
        executable='steering_calibration_node.py',
        name='steering_calibration_node',
        output='screen',
        parameters=[{
            'min_steering_angle': LaunchConfiguration('min_steering_angle'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'steering_increment': LaunchConfiguration('steering_increment'),
            'test_speed': LaunchConfiguration('test_speed'),
            'test_duration': LaunchConfiguration('test_duration'),
            'settling_time': LaunchConfiguration('settling_time'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'min_turn_radius': LaunchConfiguration('min_turn_radius'),
            'max_turn_radius': LaunchConfiguration('max_turn_radius'),
            'initial_steering_gain': LaunchConfiguration('initial_steering_gain'),
            'initial_steering_offset': LaunchConfiguration('initial_steering_offset'),
            'plot_results': LaunchConfiguration('plot_results'),
            'save_calibration': LaunchConfiguration('save_calibration'),
            'output_directory': LaunchConfiguration('output_directory'),
            'use_least_squares_circle': LaunchConfiguration('use_least_squares_circle'),
        }],
        remappings=[
            ('/drive', '/drive'),
            ('/odom', '/odom'),
        ]
    )

    return LaunchDescription([
        declare_min_steering_angle,
        declare_max_steering_angle,
        declare_steering_increment,
        declare_test_speed,
        declare_test_duration,
        declare_settling_time,
        declare_wheelbase,
        declare_min_turn_radius,
        declare_max_turn_radius,
        declare_initial_steering_gain,
        declare_initial_steering_offset,
        declare_plot_results,
        declare_save_calibration,
        declare_output_directory,
        declare_use_least_squares_circle,
        steering_calibration_node,
    ])