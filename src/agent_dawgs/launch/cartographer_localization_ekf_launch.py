
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression, PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=os.path.join(agent_dawgs_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
        default='cartographer_localization_ekf_2d.lua')

    rviz_config_dir = os.path.join(get_package_share_directory('agent_dawgs'),
        'rviz', 'cartographer_dawgs.rviz')


    DeclareLaunchArgument(
        'state_file_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun'
        # '/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps'
    ),
    DeclareLaunchArgument(
        'state_file_name',
        default_value='mohyun_1016_m'
    ),
    state_file_directory = LaunchConfiguration('state_file_directory')
    state_file_name = LaunchConfiguration('state_file_name')


    # Join directory and filename (with extension)
    state_file = LaunchConfiguration('state_file',
        default= '/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1017/mohyun_1017_2map.pbstream'),


    yaml_file = LaunchConfiguration('yaml_file',
        default= '/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1017/mohyun_1017_2map.yaml'),
#   PathJoinSubstitution([state_file_directory, state_file_name, '.pbstream']),


    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            # parameters=[{'use_sim_time': use_sim_time,}],
            parameters=[{'use_sim_time': use_sim_time,
                        'load_frozen_state': True,
                        'start_trajectory_with_default_topics': False}],
            arguments=['--configuration_directory', cartographer_config_dir,
                       '--configuration_basename', configuration_basename,
                       '--load_state_filename', state_file,
                        '--minloglevel', '2',],     # 0 for info logging
            remappings=[('odom', 'odometry/filtered'),],
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cartographer_occupancy_grid_ekf_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': use_sim_time,
                        'yaml_filename': yaml_file,}],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),

        Node(
            package='agent_dawgs',
            executable='initialpose_bridge.py',
            name='initialpose_bridge',
            output='screen'),

    ])