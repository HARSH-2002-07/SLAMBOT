"""Launch the fleet_coordinator node only.

Assumes the multi-robot simulation + per-robot Nav2 stacks are already running
(see multi_robot.launch.py). This file is intentionally minimal so the
coordinator can be restarted without touching the simulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("multi_robot_coordinator")
    default_tasks = os.path.join(pkg, "config", "tasks.yaml")

    task_file = LaunchConfiguration("task_file")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    max_retries = LaunchConfiguration("max_retries", default="1")
    dispatch_hz = LaunchConfiguration("dispatch_hz", default="2.0")

    coordinator = Node(
        package="multi_robot_coordinator",
        executable="fleet_coordinator",
        name="fleet_coordinator",
        output="screen",
        parameters=[{
            "robot_names": ["robot_1", "robot_2"],
            "task_file": task_file,
            "max_retries": max_retries,
            "dispatch_hz": dispatch_hz,
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("task_file", default_value=default_tasks),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("max_retries", default_value="1"),
        DeclareLaunchArgument("dispatch_hz", default_value="2.0"),
        coordinator,
    ])
