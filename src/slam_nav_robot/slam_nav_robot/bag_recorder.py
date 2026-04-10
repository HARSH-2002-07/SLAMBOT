#!/usr/bin/env python3
"""
Bag recorder for SLAM mapping sessions.
Records /scan, /odom, /tf, /tf_static, /map into a timestamped bag.

Usage:
    ros2 run slam_nav_robot bag_recorder
    Ctrl+C to stop recording.
"""

import os
import subprocess
import signal
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node


TOPICS = [
    '/scan',
    '/odom',
    '/tf',
    '/tf_static',
    '/map',
    '/robot_description',
    '/cmd_vel',
]


class BagRecorderNode(Node):

    def __init__(self):
        super().__init__('bag_recorder_node')

        save_dir = os.path.expanduser('~/slam_nav_ws/bags')
        os.makedirs(save_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(save_dir, f'slam_session_{timestamp}')

        cmd = ['ros2', 'bag', 'record', '-o', bag_path] + TOPICS

        self.get_logger().info(
            f'Recording bag to: {bag_path}\n'
            f'Topics: {", ".join(TOPICS)}\n'
            f'Press Ctrl+C to stop.'
        )

        self.process = subprocess.Popen(cmd)

        # Forward SIGINT so Ctrl+C closes the bag cleanly
        signal.signal(signal.SIGINT, self._stop)

    def _stop(self, sig, frame):
        self.get_logger().info('Stopping bag recording...')
        self.process.send_signal(signal.SIGINT)
        self.process.wait()
        self.get_logger().info('Bag saved.')
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()