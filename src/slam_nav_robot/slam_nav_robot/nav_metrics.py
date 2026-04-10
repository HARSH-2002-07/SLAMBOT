#!/usr/bin/env python3
"""
Localisation accuracy monitor.
Compares AMCL estimated pose vs Gazebo ground truth (model states).
Logs error in cm — target is under 5cm average.

Usage:
    ros2 run slam_nav_robot nav_metrics
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates


class NavMetrics(Node):

    def __init__(self):
        super().__init__('nav_metrics')

        self._amcl_x = None
        self._amcl_y = None
        self._gt_x = None
        self._gt_y = None
        self._errors = []

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            10
        )
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self._gt_cb,
            10
        )

        # Log every 2 seconds
        self.create_timer(2.0, self._log_error)

        self.get_logger().info(
            'Nav metrics running. Logs localisation error every 2s.\n'
            'Target: average error < 5cm'
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_x = msg.pose.pose.position.x
        self._amcl_y = msg.pose.pose.position.y

    def _gt_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index('slam_bot')
            self._gt_x = msg.pose[idx].position.x
            self._gt_y = msg.pose[idx].position.y
        except ValueError:
            pass   # model not yet spawned

    def _log_error(self):
        if None in (self._amcl_x, self._amcl_y,
                    self._gt_x, self._gt_y):
            return

        error_m = math.hypot(self._amcl_x - self._gt_x,
                             self._amcl_y - self._gt_y)
        error_cm = error_m * 100.0
        self._errors.append(error_cm)
        avg_cm = sum(self._errors) / len(self._errors)

        self.get_logger().info(
            f'Localisation error — '
            f'current: {error_cm:.1f}cm  '
            f'average: {avg_cm:.1f}cm  '
            f'samples: {len(self._errors)}'
        )

        if avg_cm > 10.0 and len(self._errors) > 5:
            self.get_logger().warn(
                'Average error > 10cm. '
                'Consider increasing AMCL particles or driving slower.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        errors = node._errors
        if errors:
            node.get_logger().info(
                f'Final average localisation error: '
                f'{sum(errors)/len(errors):.1f}cm '
                f'over {len(errors)} samples'
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
