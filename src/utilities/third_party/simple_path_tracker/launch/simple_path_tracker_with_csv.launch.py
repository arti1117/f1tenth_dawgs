import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('simple_path_tracker')

    # Get config file path
    config_file = os.path.join(pkg_dir, 'config', 'simple_path_tracker.yaml')

    # Declare launch arguments
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='',
        description='Path to CSV file containing track waypoints (required)'
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

    rviz_config_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Simple path tracker node
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

    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'simple_path_tracker.rviz')],
        condition=LaunchConfiguration('rviz')
    )

    return LaunchDescription([
        csv_file_arg,
        config_file_arg,
        use_sim_time_arg,
        rviz_config_arg,
        simple_path_tracker_node
    ])