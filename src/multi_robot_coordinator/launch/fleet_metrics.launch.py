"""Launch the fleet_metrics node only.

Expects the coordinator (and therefore /fleet_coordinator/status) to be
running. Safe to restart independently.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    publish_hz = LaunchConfiguration("publish_hz", default="1.0")

    metrics = Node(
        package="multi_robot_coordinator",
        executable="fleet_metrics",
        name="fleet_metrics",
        output="screen",
        parameters=[{
            "robot_names": ["robot_1", "robot_2"],
            "publish_hz": publish_hz,
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        metrics,
    ])
