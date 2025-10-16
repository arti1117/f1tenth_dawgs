#!/usr/bin/env python3
"""
Launch file for ERPM Gain Calibration

This launch file starts the ERPM calibration node to determine the optimal
relationship between commanded speed and motor ERPM.

Usage:
    ros2 launch vehicle_calibration erpm_calibration.launch.py
    ros2 launch vehicle_calibration erpm_calibration.launch.py max_speed:=4.0 initial_erpm_gain:=3500
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    declare_min_speed = DeclareLaunchArgument(
        'min_speed',
        default_value='0.8',
        description='Minimum calibration speed (m/s)'
    )

    declare_max_speed = DeclareLaunchArgument(
        'max_speed',
        default_value='4.0',
        description='Maximum calibration speed (m/s)'
    )

    declare_speed_increment = DeclareLaunchArgument(
        'speed_increment',
        default_value='0.3',
        description='Speed increment per test (m/s)'
    )

    declare_test_duration = DeclareLaunchArgument(
        'test_duration',
        default_value='8.0',
        description='Duration for each speed test (s)'
    )

    declare_settling_time = DeclareLaunchArgument(
        'settling_time',
        default_value='2.0',
        description='Time to let speed settle before recording (s)'
    )

    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.33',
        description='Vehicle wheelbase (m)'
    )

    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
        description='Wheel radius (m)'
    )

    declare_initial_erpm_gain = DeclareLaunchArgument(
        'initial_erpm_gain',
        default_value='4000',
        description='Initial ERPM gain estimate'
    )

    declare_motor_poles = DeclareLaunchArgument(
        'motor_poles',
        default_value='4',
        description='Number of motor poles'
    )

    declare_max_erpm = DeclareLaunchArgument(
        'max_erpm',
        default_value='20000',
        description='Maximum ERPM limit'
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

    # ERPM Calibration Node
    erpm_calibration_node = Node(
        package='vehicle_calibration',
        executable='erpm_calibration_node.py',
        name='erpm_calibration_node',
        output='screen',
        parameters=[{
            'min_speed': LaunchConfiguration('min_speed'),
            'max_speed': LaunchConfiguration('max_speed'),
            'speed_increment': LaunchConfiguration('speed_increment'),
            'test_duration': LaunchConfiguration('test_duration'),
            'settling_time': LaunchConfiguration('settling_time'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'initial_erpm_gain': LaunchConfiguration('initial_erpm_gain'),
            'motor_poles': LaunchConfiguration('motor_poles'),
            'max_erpm': LaunchConfiguration('max_erpm'),
            'plot_results': LaunchConfiguration('plot_results'),
            'save_calibration': LaunchConfiguration('save_calibration'),
            'output_directory': LaunchConfiguration('output_directory'),
        }],
        remappings=[
            ('/drive', '/drive'),
            ('/odom', '/odom'),
            ('/sensors/core', '/sensors/core'),
            ('/joint_states', '/joint_states'),
        ]
    )

    return LaunchDescription([
        declare_min_speed,
        declare_max_speed,
        declare_speed_increment,
        declare_test_duration,
        declare_settling_time,
        declare_wheelbase,
        declare_wheel_radius,
        declare_initial_erpm_gain,
        declare_motor_poles,
        declare_max_erpm,
        declare_plot_results,
        declare_save_calibration,
        declare_output_directory,
        erpm_calibration_node,
    ])