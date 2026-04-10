#!/usr/bin/env python3
"""
Waypoint navigator for the SLAM navigation robot.
Sends a sequence of goals to Nav2 and logs results.

Usage:
    ros2 run slam_nav_robot waypoint_navigator
"""

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# ── Waypoints (x, y, yaw_degrees) ─────────────────────────────────────────
# These are positions inside room.world. Adjust if your map origin differs.
WAYPOINTS: List[Tuple[float, float, float]] = [
    ( 1.5,  1.5,   0.0),   # far corner near box_1
    ( 2.0, -1.5,  90.0),   # near cylinder_1
    (-1.5, -1.0, 180.0),   # near box_2
    (-2.0, -2.0, 270.0),   # back to start area
]


def yaw_to_quaternion(yaw_deg: float):
    """Convert yaw in degrees to a quaternion (z, w only for 2D)."""
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)   # qz, qw


def make_pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(yaw_deg)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._results = []   # (waypoint_index, success, duration_s)

    def navigate_to(self, index: int, x: float, y: float, yaw: float) -> bool:
        """Send one goal, block until complete. Returns True on success."""
        self.get_logger().info(
            f'Waypoint {index+1}/{len(WAYPOINTS)}: '
            f'({x:.2f}, {y:.2f}, {yaw:.0f}°)'
        )

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        t_start = time.time()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal {index+1} rejected by Nav2')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        duration = time.time() - t_start
        status = result_future.result().status
        success = (status == GoalStatus.STATUS_SUCCEEDED)

        if success:
            self.get_logger().info(
                f'Waypoint {index+1} reached in {duration:.1f}s')
        else:
            self.get_logger().warn(
                f'Waypoint {index+1} FAILED (status={status}) '
                f'after {duration:.1f}s')

        self._results.append((index + 1, success, duration))
        return success

    def run_mission(self):
        """Execute all waypoints and print a summary."""
        self.get_logger().info(
            f'Starting mission: {len(WAYPOINTS)} waypoints')

        # Wait for Nav2 lifecycle to be active
        self.get_logger().info('Waiting 3s for Nav2 to be ready...')
        time.sleep(3.0)

        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            self.navigate_to(i, x, y, yaw)
            # Short pause between waypoints
            time.sleep(1.0)

        self._print_summary()

    def _print_summary(self):
        succeeded = sum(1 for _, ok, _ in self._results if ok)
        total     = len(self._results)
        total_t   = sum(t for _, _, t in self._results)

        self.get_logger().info('─' * 50)
        self.get_logger().info(f'Mission complete: {succeeded}/{total} waypoints reached')
        self.get_logger().info(f'Total time: {total_t:.1f}s')
        self.get_logger().info('─' * 50)

        for idx, ok, dur in self._results:
            status = 'OK' if ok else 'FAIL'
            self.get_logger().info(
                f'  Waypoint {idx}: {status}  ({dur:.1f}s)')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()