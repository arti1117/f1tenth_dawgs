from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare arguments
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1017/mohyun_1017_2map_iqp_0.35.csv',
        description='Path to racing path CSV file'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for published path'
    )

    map_image_arg = DeclareLaunchArgument(
        'map_image',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1017/mohyun_1017_2map.pgm',
        description='Path to map image file (pgm/png)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1017/mohyun_1017_2map.yaml',
        description='Path to map YAML file with origin/resolution info'
    )

    # Path editor node
    path_editor_node = Node(
        package='path_editor',
        executable='path_editor_node.py',
        name='path_editor',
        output='screen',
        parameters=[{
            'csv_file_path': LaunchConfiguration('csv_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'map_image_path': LaunchConfiguration('map_image'),
            'map_yaml_path': LaunchConfiguration('map_yaml'),
        }]
    )

    return LaunchDescription([
        csv_file_arg,
        frame_id_arg,
        map_image_arg,
        map_yaml_arg,
        path_editor_node,
    ])
