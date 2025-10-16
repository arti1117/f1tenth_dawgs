import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    ekf_iahrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'ekf_iahrs_launch.py')
        )
    )

    cartographer_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'cartographer_ekf_launch.py')
        )
    )

    cartographer_occupancy_grid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'cartographer_occupancy_grid_launch.py')
        )
    )

    f1tenth_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('f1tenth_stack'), 'launch', 'bringup_launch.py')
        )
    )

    # tf_publisher_node = Node(
    #     package="iahrs_driver",
    #     executable="iahrs_tf_publisher",
    #     name='iahrs_tf_publisher'
    # )
    # rviz_config_path = os.path.join(
    #     get_package_share_directory('iahrs_driver'),
    #     'rviz',
    #     'iahrs_debug.rviz'
    # )
    # iahrs_rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2_node',
    #     output='screen',
    #     arguments=['-d', rviz_config_path]
    # )

    ld.add_action(ekf_iahrs_launch)
    ld.add_action(cartographer_ekf_launch)
    # ld.add_action(cartographer_occupancy_grid_launch)
    ld.add_action(f1tenth_system_launch)
    return ld