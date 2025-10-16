from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Launch arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='levine/levine_blocked_map',
        description='Name of the map folder under global_planner/maps/'
    )

    show_plots_arg = DeclareLaunchArgument(
        'show_plots',
        default_value='False',
        description='Show matplotlib figures (True/False)'
    )

    create_map_arg = DeclareLaunchArgument(
        'create_map',
        default_value='False',
        description='Whether to create map centerline from png (True/False)'
    )

    # Get parameter file
    pkg_dir = os.path.join(
        # os.getenv('COLCON_PREFIX_PATH', '').split(':')[0],
        get_package_share_directory('global_planner'),
        'share',
        'global_planner'
    )
    param_file = os.path.join(get_package_share_directory('global_planner'), 'config', 'global_planner_params.yaml')

    node = Node(
        package='global_planner',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[param_file, {
            'map_name': LaunchConfiguration('map_name'),
            'show_plots': LaunchConfiguration('show_plots'),
            'create_map': LaunchConfiguration('create_map')
        }]
    )

    return LaunchDescription([
        map_name_arg,
        show_plots_arg,
        create_map_arg,
        node
    ])
