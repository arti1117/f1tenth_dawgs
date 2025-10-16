
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim_cmaes.yaml'
        ) # sim_reactive.yaml or sim.yaml
    config_dict = yaml.safe_load(open(config, 'r'))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'),
        'launch', 'gym_bridge_pp.rviz')]
    )


    ld.add_action(rviz_node)

    return ld