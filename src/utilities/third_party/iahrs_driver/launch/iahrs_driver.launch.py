import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('iahrs_driver'),
        'config',
        'params.yaml'
    )

    iahrs_node = Node(
        package="iahrs_driver",
        executable="iahrs_driver",
        name='iahrs_driver',
        parameters=[config]

    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_iahrs',
        arguments=['0.28', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'iahrs']
    )

    ld.add_action(iahrs_node)
    ld.add_action(static_tf_node)
    return ld