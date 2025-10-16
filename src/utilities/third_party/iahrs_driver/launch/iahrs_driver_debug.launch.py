import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():
    ld = LaunchDescription()

    iahrs_driver_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('iahrs_driver'), 'launch', 'iahrs_driver.launch.py')
        )
    )

    tf_publisher_node = Node(
        package="iahrs_driver",
        executable="iahrs_tf_publisher",
        name='iahrs_tf_publisher'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('iahrs_driver'),
        'rviz',
        'iahrs_debug.rviz'
    )

    iahrs_rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            arguments=['-d', rviz_config_path]
        )


    ld.add_action(iahrs_driver_launch)
    ld.add_action(tf_publisher_node)
    ld.add_action(iahrs_rviz_node)
    return ld