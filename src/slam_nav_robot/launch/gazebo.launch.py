import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('slam_nav_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── Paths ──────────────────────────────────────────────────
    xacro_file    = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    world_file    = os.path.join(pkg_share, 'worlds', 'room.world')

    # ── Launch Arguments ───────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    x_arg = DeclareLaunchArgument('x', default_value='-2.0',
                                  description='Robot spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='-2.0',
                                  description='Robot spawn Y position')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Robot Description (XACRO → URDF string) ────────────────
    robot_description = Command(['xacro ', xacro_file])

    # ── Nodes ──────────────────────────────────────────────────

    # 1. Robot State Publisher — publishes TF from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # 2. Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    # 3. Spawn the robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'slam_bot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.05',
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        x_arg,
        y_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
    ])