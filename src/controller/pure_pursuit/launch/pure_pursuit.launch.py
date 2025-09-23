import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'pure_pursuit'
    param_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            output='screen',
            parameters=[param_file]
        )
    ])
