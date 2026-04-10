#!/usr/bin/env python3
"""
Map saver utility.
Run this after you have finished driving and the map looks clean.

Usage:
    ros2 run slam_nav_robot map_saver
    ros2 run slam_nav_robot map_saver --ros-args -p map_name:=my_room
"""

import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class MapSaverNode(Node):

    def __init__(self):
        super().__init__('map_saver_node')

        self.declare_parameter('map_name', '')
        self.declare_parameter('save_dir', os.path.expanduser('~/slam_nav_ws/maps'))

        map_name = self.get_parameter('map_name').value
        save_dir = self.get_parameter('save_dir').value

        # Auto-generate name if not provided
        if not map_name:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            map_name = f'map_{timestamp}'

        os.makedirs(save_dir, exist_ok=True)
        map_path = os.path.join(save_dir, map_name)

        self.get_logger().info(f'Saving map to: {map_path}')

        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_path,
             '--ros-args', '-p', 'save_map_timeout:=5.0'],
            capture_output=True, text=True
        )

        if result.returncode == 0:
            self.get_logger().info(
                f'Map saved successfully!\n'
                f'  {map_path}.pgm\n'
                f'  {map_path}.yaml'
            )
        else:
            self.get_logger().error(
                f'Map save failed:\n{result.stderr}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()