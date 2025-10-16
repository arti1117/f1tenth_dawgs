#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for toggle-based joystick teleop"""

    # Declare launch arguments
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='15.0',
        description='Maximum speed in m/s'
    )

    max_steering_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.41',
        description='Maximum steering angle in radians'
    )

    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='0',
        description='Joystick device ID'
    )

    # Joy node for reading joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
            'coalesce_interval': 0.01
        }],
        output='screen'
    )

    # Toggle teleop node
    toggle_teleop_node = Node(
        package='joy_teleop',
        executable='joy_toggle_teleop',
        name='joy_toggle_teleop',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'speed_axis': 1,  # Left stick Y axis
            'steering_axis': 0,  # Left stick X axis
            'toggle_button': 5,  # RB button
            'emergency_stop_button': 0,  # A button
            'deadzone': 0.1
        }],
        remappings=[
            ('teleop', 'teleop'),  # Publish to standard teleop topic (change 2nd )
            ('joy', 'joy')  # Subscribe to joy topic
        ],
        output='screen'
    )

    return LaunchDescription([
        max_speed_arg,
        max_steering_arg,
        joy_device_arg,
        joy_node,
        toggle_teleop_node
    ])