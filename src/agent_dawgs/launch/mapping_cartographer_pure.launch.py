import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    mapping_config = os.path.join(
        get_package_share_directory('agent_dawgs'),
        'config',
        'dawgs',
        'mapping_vesc.yaml'
    )

    f1tenth_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'bringup_launch.py')
        ),
        launch_arguments={
            'vesc_config': mapping_config
        }.items()
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'ekf_launch.py')
        )
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'cartographer_pure_launch.py')
        )
    )

    cartographer_occupancy_grid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'cartographer_occupancy_grid_launch.py')
        )
    )



    ld.add_action(f1tenth_system_launch)
    # ld.add_action(ekf_launch)
    ld.add_action(cartographer_launch)
    # ld.add_action(cartographer_occupancy_grid_launch)
    return ld