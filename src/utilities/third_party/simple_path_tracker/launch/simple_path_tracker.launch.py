import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('simple_path_tracker')

    # Get config file path
    config_file = os.path.join(pkg_dir, 'config', 'simple_path_tracker.yaml')

    # Declare launch arguments
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='',
        description='Path to CSV file containing track waypoints'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    # Create node
    simple_path_tracker_node = Node(
        package='simple_path_tracker',
        executable='simple_path_tracker_node',
        name='simple_path_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'csv_file_path': LaunchConfiguration('csv_file')}
        ],
        remappings=[
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/global_path', '/global_path'),
            ('/local_path', '/local_path')
        ]
    )

    return LaunchDescription([
        csv_file_arg,
        config_file_arg,
        use_sim_time_arg,
        simple_path_tracker_node
    ])