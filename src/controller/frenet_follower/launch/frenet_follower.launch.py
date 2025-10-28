from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('frenet_follower')

    # Path to config file
    config_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    # Frenet follower node
    frenet_follower_node = Node(
        package='frenet_follower',
        executable='frenet_follower_node',
        name='frenet_follower_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Remap topics if needed
        ]
    )

    return LaunchDescription([
        frenet_follower_node
    ])
