"""Week 9 demo launch: single robot with RGB-D camera in vlm_room.world.

Publishes:
    /scan, /odom, /tf
    /camera/image_raw, /camera/depth/image_raw
    /camera/camera_info, /camera/depth/camera_info, /camera/points

Intended for Week 10 integration where the VLM node subscribes to
/camera/image_raw and issues NavigateToPose goals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vlm_nav_robot')

    world_path = os.path.join(pkg, 'worlds', 'vlm_room.world')
    urdf_path = os.path.join(pkg, 'description', 'robot.urdf.xacro')

    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

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

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_entity,
    ])
