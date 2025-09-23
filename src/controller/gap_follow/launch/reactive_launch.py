from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    reactive_config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'gap_follow.yaml'
        )

    ld = LaunchDescription()

    reactive_node = Node(
        package='gap_follow',
        executable='reactive_node',
        name='reactive_node',
        parameters=[reactive_config]
    )

    ld.add_action(reactive_node)

    return ld
