#!/usr/bin/env python3
"""
Launch file for offline global planner node.

This node generates trajectories from PNG/YAML map files WITHOUT requiring
ROS topics (/map, /car_state/pose).

Usage:
    # Basic usage
    ros2 launch global_planner global_planner_offline.launch.py

    # Specify different map
    ros2 launch global_planner global_planner_offline.launch.py \
        map_name:=GLC_smile_small \
        map_dir:=/path/to/maps/GLC_smile_small

    # Reverse direction
    ros2 launch global_planner global_planner_offline.launch.py \
        map_name:=hangar_1905_v0 \
        reverse_mapping:=true

    # Custom safety widths
    ros2 launch global_planner global_planner_offline.launch.py \
        map_name:=my_track \
        safety_width:=0.30 \
        safety_width_sp:=0.25
"""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description for offline global planner."""

    # Get package directory
    pkg_dir = get_package_share_directory('global_planner')
    default_config = os.path.join(pkg_dir, 'config', 'global_planner_offline_params.yaml')

    # Default map directory (can be overridden by parameter file)
    default_map_dir = os.path.join(
        os.path.dirname(pkg_dir),
        'stack_master', 'maps', 'hangar_1905_v0'
    )

    # Launch arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='levine/levine_blocked_map',
        description='Name of the map (without extension)'
    )

    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value=default_map_dir,
        description='Full path to directory containing map PNG and YAML files'
    )

    safety_width_arg = DeclareLaunchArgument(
        'safety_width',
        default_value='0.25',
        description='Safety width for IQP trajectory in meters'
    )

    safety_width_sp_arg = DeclareLaunchArgument(
        'safety_width_sp',
        default_value='0.20',
        description='Safety width for shortest path trajectory in meters'
    )

    reverse_mapping_arg = DeclareLaunchArgument(
        'reverse_mapping',
        default_value='false',
        description='Reverse the raceline direction (true/false)'
    )

    show_plots_arg = DeclareLaunchArgument(
        'show_plots',
        default_value='true',
        description='Show matplotlib plots during generation (true/false)'
    )

    # Node
    offline_planner_node = Node(
        package='global_planner',
        executable='global_planner_offline',
        name='global_planner_offline_node',
        output='screen',
        parameters=[
            default_config,
            {
                'map_name': LaunchConfiguration('map_name'),
                'map_dir': LaunchConfiguration('map_dir'),
                'safety_width': LaunchConfiguration('safety_width'),
                'safety_width_sp': LaunchConfiguration('safety_width_sp'),
                'reverse_mapping': LaunchConfiguration('reverse_mapping'),
                'show_plots': LaunchConfiguration('show_plots'),
            }
        ]
    )

    return LaunchDescription([
        map_name_arg,
        map_dir_arg,
        safety_width_arg,
        safety_width_sp_arg,
        reverse_mapping_arg,
        show_plots_arg,
        offline_planner_node
    ])
