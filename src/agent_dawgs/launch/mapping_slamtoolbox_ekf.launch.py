import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_dir = os.path.join(get_package_share_directory('agent_dawgs'),
        'rviz', 'cartographer_dawgs.rviz')

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

    slamtoolbox_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agent_dawgs'), 'launch', 'slamtoolbox_ekf_launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    ld.add_action(f1tenth_system_launch)
    ld.add_action(ekf_launch)
    ld.add_action(slamtoolbox_ekf_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)
    return ld