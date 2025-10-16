# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    carto_config_dir = LaunchConfiguration('carto_config_dir',
        default=os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config')
    )
    carto_config_name = LaunchConfiguration('carto_config_name',
        default = 'f1tenth_carto_lds_2d.lua'
    )


    # SLAM
    resolution_la = DeclareLaunchArgument(
        'resolution',
        default_value=resolution,
        description='Resolution of a grid cell in the published occupancy grid')
    publish_period_sec_la = DeclareLaunchArgument(
        'publish_period_sec',
        default_value=publish_period_sec,
        description='OccupancyGrid publishing period')
    carto_config_dir_la = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=carto_config_dir,
        description='Full path to config file to load')
    carto_config_name_la = DeclareLaunchArgument(
        'configuration_basename',
        default_value=carto_config_name,
        description='Name of lua file for cartographer')

    ld = LaunchDescription([carto_config_dir_la, carto_config_name_la,
                            resolution_la, publish_period_sec_la])


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='log',
        parameters=[LaunchConfiguration('slam_config')],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_stack'), 'launch', 'carto_mapping.rviz')]
    )
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', carto_config_dir,
                   '-configuration_basename', carto_config_name]
    )
    occup_grid_node = Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    # finalize
    ld.add_action(occup_grid_node)

    return ld
