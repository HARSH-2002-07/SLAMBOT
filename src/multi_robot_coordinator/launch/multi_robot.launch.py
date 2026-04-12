# launch/multi_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg  = get_package_share_directory('multi_robot_coordinator')
    spawn = os.path.join(pkg, 'launch', 'spawn_robot.launch.py')

    # Start Gazebo with your existing world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             os.path.join(pkg, 'worlds', 'room.world'),
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so'],
        output='screen',
    )

    robot_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn),
        launch_arguments={
            'robot_name': 'robot_1',
            'robot_ns':   'robot_1',
            'x_pose':     '-1.0',
            'y_pose':     '0.0',
        }.items(),
    )

    # Delay robot_2 by 3s — Gazebo needs robot_1 loaded first
    robot_2 = TimerAction(
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

    # Static TF: map → robot_1/odom and map → robot_2/odom
    # (AMCL will replace these once Nav2 is up in Week 6)
    static_tf_r1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_r1',
        arguments=['0', '0', '0', '0', '0', '0',
                   'map', 'robot_1/odom'],
    )
    static_tf_r2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_r2',
        arguments=['0', '0', '0', '0', '0', '0',
                   'map', 'robot_2/odom'],
    )

    return LaunchDescription([
        gazebo,
        robot_1,
        robot_2,
        static_tf_r1,
        static_tf_r2,
    ])