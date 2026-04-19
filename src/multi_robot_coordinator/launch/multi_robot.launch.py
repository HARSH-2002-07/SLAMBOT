# launch/multi_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess,
                             TimerAction, DeclareLaunchArgument)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg  = get_package_share_directory('multi_robot_coordinator')
    spawn   = os.path.join(pkg, 'launch', 'spawn_robot.launch.py')
    nav2    = os.path.join(pkg, 'launch', 'nav2_robot.launch.py')

    map_yaml = LaunchConfiguration('map_yaml')
    headless = LaunchConfiguration('headless')

    # Your existing map — point to wherever Project 1 saved it
    map_default = os.path.join(
        os.path.expanduser('~'), 'slam_nav_ws', 'maps', 'map.yaml'
    )

    world_path = os.path.join(pkg, 'worlds', 'room.world')
    gazebo_plugins = ['-s', 'libgazebo_ros_factory.so',
                      '-s', 'libgazebo_ros_init.so']

    gazebo_gui = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, *gazebo_plugins],
        output='screen',
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path, *gazebo_plugins],
        output='screen',
        condition=IfCondition(headless),
    )

    robot_1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn),
        launch_arguments={
            'robot_name': 'robot_1',
            'robot_ns':   'robot_1',
            'x_pose':     '-1.0',
            'y_pose':     '0.0',
        }.items(),
    )

    robot_2_spawn = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn),
            launch_arguments={
                'robot_name': 'robot_2',
                'robot_ns':   'robot_2',
                'x_pose':     '1.0',
                'y_pose':     '0.0',
            }.items(),
        )]
    )

    # Nav2 delayed — wait for both robots and Gazebo to stabilise
    robot_1_nav2 = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2),
            launch_arguments={
                'namespace': 'robot_1',
                'map_yaml':  map_yaml,
            }.items(),
        )]
    )

    robot_2_nav2 = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2),
            launch_arguments={
                'namespace': 'robot_2',
                'map_yaml':  map_yaml,
            }.items(),
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=map_default,
            description='Full path to map yaml'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='If true, run gzserver only (no Gazebo GUI client)'
        ),
        gazebo_gui,
        gazebo_headless,
        robot_1_spawn,
        robot_2_spawn,
        robot_1_nav2,
        robot_2_nav2,
    ])