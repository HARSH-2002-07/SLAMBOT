# launch/spawn_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('multi_robot_coordinator')

    robot_name = LaunchConfiguration('robot_name')
    robot_ns   = LaunchConfiguration('robot_ns')
    x_pose     = LaunchConfiguration('x_pose')
    y_pose     = LaunchConfiguration('y_pose')

    # ← Wrap in ParameterValue with value_type=str
    robot_description = ParameterValue(
        Command([
            'xacro ', os.path.join(pkg, 'description', 'robot.urdf.xacro'),
            ' robot_name:=', robot_name,
            ' robot_ns:=',   robot_ns,
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot_1'),
        DeclareLaunchArgument('robot_ns',   default_value='/robot_1'),
        DeclareLaunchArgument('x_pose',     default_value='0.0'),
        DeclareLaunchArgument('y_pose',     default_value='0.0'),

        # robot_state_publisher — one per robot, scoped to its namespace
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_ns,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'frame_prefix': [robot_ns, '/'],
            }],
        ),

        # Spawn into Gazebo at the given pose
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', [robot_ns, '/robot_description'],
                '-entity', robot_name,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
            ],
            output='screen',
        ),
    ])