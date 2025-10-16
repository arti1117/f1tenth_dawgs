#!/usr/bin/env python3
"""
Master Launch file for Vehicle Calibration Suite

This launch file provides options to start individual calibration nodes
or run them sequentially for complete vehicle calibration.

Usage:
    # Run all calibrations
    ros2 launch vehicle_calibration vehicle_calibration.launch.py run_all:=true

    # Run specific calibrations
    ros2 launch vehicle_calibration vehicle_calibration.launch.py run_friction_circle:=true
    ros2 launch vehicle_calibration vehicle_calibration.launch.py run_erpm:=true
    ros2 launch vehicle_calibration vehicle_calibration.launch.py run_steering:=true

    # Run with custom parameters
    ros2 launch vehicle_calibration vehicle_calibration.launch.py run_erpm:=true max_speed:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('vehicle_calibration')

    # Declare launch arguments for calibration selection
    declare_run_all = DeclareLaunchArgument(
        'run_all',
        default_value='false',
        description='Run all calibration nodes'
    )

    declare_run_friction_circle = DeclareLaunchArgument(
        'run_friction_circle',
        default_value='false',
        description='Run friction circle test'
    )

    declare_run_erpm = DeclareLaunchArgument(
        'run_erpm',
        default_value='false',
        description='Run ERPM calibration'
    )

    declare_run_steering = DeclareLaunchArgument(
        'run_steering',
        default_value='false',
        description='Run steering calibration'
    )

    # Common parameters
    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.33',
        description='Vehicle wheelbase (m)'
    )

    declare_max_speed = DeclareLaunchArgument(
        'max_speed',
        default_value='3.0',
        description='Maximum test speed (m/s)'
    )

    declare_output_directory = DeclareLaunchArgument(
        'output_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/data/calibration',
        description='Output directory for calibration data'
    )

    # Friction Circle Launch
    friction_circle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'friction_circle.launch.py'
            ])
        ]),
        launch_arguments={
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_speed': LaunchConfiguration('max_speed'),
            'output_directory': LaunchConfiguration('output_directory'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_friction_circle'))
    )

    # ERPM Calibration Launch
    erpm_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'erpm_calibration.launch.py'
            ])
        ]),
        launch_arguments={
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_speed': LaunchConfiguration('max_speed'),
            'output_directory': LaunchConfiguration('output_directory'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_erpm'))
    )

    # Steering Calibration Launch
    steering_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'steering_calibration.launch.py'
            ])
        ]),
        launch_arguments={
            'wheelbase': LaunchConfiguration('wheelbase'),
            'output_directory': LaunchConfiguration('output_directory'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_steering'))
    )

    # Calibration Control Node - provides unified control interface
    calibration_control_node = Node(
        package='vehicle_calibration',
        executable='calibration_control_node.py',
        name='calibration_control_node',
        output='screen',
        parameters=[{
            'run_all': LaunchConfiguration('run_all'),
            'run_friction_circle': LaunchConfiguration('run_friction_circle'),
            'run_erpm': LaunchConfiguration('run_erpm'),
            'run_steering': LaunchConfiguration('run_steering'),
            'output_directory': LaunchConfiguration('output_directory'),
        }],
        condition=IfCondition(LaunchConfiguration('run_all'))
    )

    return LaunchDescription([
        # Launch arguments
        declare_run_all,
        declare_run_friction_circle,
        declare_run_erpm,
        declare_run_steering,
        declare_wheelbase,
        declare_max_speed,
        declare_output_directory,

        # Conditional launches
        friction_circle_launch,
        erpm_calibration_launch,
        steering_calibration_launch,
        calibration_control_node,
    ])