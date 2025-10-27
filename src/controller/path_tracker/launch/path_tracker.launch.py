import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'path_tracker'
    param_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'tracker_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='path_tracker',
            executable='path_tracker_node',
            name='path_tracker_node',
            output='screen',
            parameters=[param_file]
            # , remappings=[
            #     ('/odom', '/odometry/filtered'),
            # ]
            # , remappings=[
            #     ('/odom', '/ego_racecar/odom'),
            # ]
            , remappings=[
                ('/odom', '/pf/pose/odom'),
            ]
        )
    ])
