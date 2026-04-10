import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import glob


def get_latest_map(maps_dir: str) -> str:
    """Return the most recently saved map yaml."""
    files = sorted(glob.glob(os.path.join(maps_dir, '*.yaml')))
    if not files:
        raise FileNotFoundError(
            f'No map yaml found in {maps_dir}. '
            f'Run map_saver first (Week 2).'
        )
    return files[-1]


def generate_launch_description():

    pkg_share   = get_package_share_directory('slam_nav_robot')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    maps_dir    = os.path.expanduser('~/slam_nav_ws/maps')
    map_yaml    = get_latest_map(maps_dir)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Map server — loads your saved .pgm map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time,
             'yaml_filename': map_yaml}
        ]
    )

    # 2. AMCL — localises robot inside the loaded map
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. Nav2 controller server (DWB local planner)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # 4. Planner server (NavFn global planner)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # 5. Behaviour server (spin, backup, wait)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # 6. BT Navigator — orchestrates everything via behaviour trees
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # 7. Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # 8. Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # 9. Lifecycle manager — brings all Nav2 nodes up in order
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
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
                'velocity_smoother',
            ]
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])