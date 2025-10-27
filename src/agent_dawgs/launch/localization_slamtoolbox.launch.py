#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_bringup = LaunchConfiguration('use_bringup', default='true')
    use_ekf = LaunchConfiguration('use_ekf', default='true')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    f1tenth_stack_prefix = get_package_share_directory('f1tenth_stack')
    rviz_config_dir = os.path.join(agent_dawgs_prefix, 'rviz', 'cartographer_dawgs.rviz')

    # F1TENTH bringup launch (without static map->odom for localization)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(f1tenth_stack_prefix, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={'publish_map_odom_tf': 'false'}.items(),
        condition=IfCondition(use_bringup)
    )

    # EKF launch for odometry/filtered
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(agent_dawgs_prefix, 'launch', 'ekf_launch.py')
        ]),
        condition=IfCondition(use_ekf)
    )

    # Map file configuration
    yaml_file = LaunchConfiguration('yaml_file',
        default='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1027/mohyun_slam_ekf.yaml'),

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_argument = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization')

    declare_use_bringup_argument = DeclareLaunchArgument(
        'use_bringup',
        default_value='true',
        description='Launch F1TENTH hardware bringup')

    declare_use_ekf_argument = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Launch EKF for odometry filtering')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(agent_dawgs_prefix,
                                   'config', 'mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    # SLAM Toolbox localization node
    start_localization_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
        ,remappings=[
            ('odom', 'odometry/filtered'),
        ],
    )

    # Lifecycle events for SLAM Toolbox - simplified for ROS2 Foxy compatibility
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_localization_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_localization_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_localization_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': yaml_file,
        }],
        output='screen')

    # Lifecycle manager for map server
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen')

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_use_rviz_argument)
    ld.add_action(declare_use_bringup_argument)
    ld.add_action(declare_use_ekf_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)

    # F1TENTH hardware bringup
    ld.add_action(bringup_launch)

    # EKF for odometry filtering
    ld.add_action(ekf_launch)

    # Nodes
    ld.add_action(start_localization_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(rviz_node)

    return ld
