import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    f1tenth_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'bringup_launch.py')
        ),
        launch_arguments={'publish_map_odom_tf': 'false'}.items()
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'ekf_launch.py')
        )
    )

    cartographer_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'cartographer_localization_ekf_launch.py')
        )
    )

    cartographer_occupancy_grid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'cartographer_occupancy_grid_ekf_launch.py')
        )
    )

    initialpose_bridge_node = Node(
        package='agent_dawgs',
        executable='initialpose_bridge.py',
        name='initialpose_bridge',
        output='screen'
    )

    ld.add_action(f1tenth_system_launch)
    ld.add_action(ekf_launch)
    ld.add_action(cartographer_ekf_launch)
    ld.add_action(initialpose_bridge_node)
    # ld.add_action(cartographer_occupancy_grid_launch)
    return ld