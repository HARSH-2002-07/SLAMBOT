# launch/nav2_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('multi_robot_coordinator')

    namespace    = LaunchConfiguration('namespace')
    map_yaml     = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file  = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # RewrittenYaml substitutes frame names per robot at launch time
    # This is the key — one params file, N robots
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'base_frame_id':   [namespace, '/base_footprint'],
            'odom_frame_id':   [namespace, '/odom'],
            'robot_base_frame':[namespace, '/base_footprint'],
            'global_frame':    [namespace, '/odom'],  # local costmap only
        },
        convert_types=True,
    )

    nav2_nodes = GroupAction(actions=[
        PushRosNamespace(namespace),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[configured_params,
                        {'yaml_filename': map_yaml,
                         'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }],
            output='screen',
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace',  default_value='robot_1'),
        DeclareLaunchArgument('map_yaml',   default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        nav2_nodes,
    ])