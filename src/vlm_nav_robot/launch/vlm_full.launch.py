"""Week 10 Part B — full stack: sim + map_server + Nav2 + vlm_grounder.

Launch args:
    map_yaml    path to the SLAM-saved map .yaml (default: maps/vlm_room_20260420_210418.yaml)
    headless    gzserver vs gazebo GUI (default: false)
    x_pose, y_pose   robot spawn (default: 0.0, 0.0)

A FindAndGo service call to /vlm_grounder/find_and_go will ground the target
via Gemini, back-project to map frame, and issue a NavigateToPose goal — so
the robot actually drives.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vlm_nav_robot')

    world_path = os.path.join(pkg, 'worlds', 'vlm_room.world')
    urdf_path = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    default_map = os.path.expanduser('~/slam_nav_ws/maps/vlm_room_20260420_210418.yaml')

    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    map_yaml = LaunchConfiguration('map_yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    gazebo_gui = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    gazebo_headless = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'vlm_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05',
        ],
        output='screen',
    )

    nav2_lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    nav2_nodes = [
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[nav2_params,
                         {'yaml_filename': map_yaml,
                          'use_sim_time': True}]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': nav2_lifecycle_nodes,
             }]),
    ]

    # Delay Nav2 until Gazebo+robot are up and publishing TF.
    nav2_delayed = TimerAction(period=6.0, actions=nav2_nodes)

    vlm_grounder = Node(
        package='vlm_nav_robot',
        executable='vlm_grounder',
        name='vlm_grounder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'confidence_threshold': 0.5,
            'standoff_m': 0.3,
        }],
    )
    # Give Nav2 time to activate before the grounder tries to connect to the action server.
    vlm_delayed = TimerAction(period=12.0, actions=[vlm_grounder])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_entity,
        nav2_delayed,
        vlm_delayed,
    ])
