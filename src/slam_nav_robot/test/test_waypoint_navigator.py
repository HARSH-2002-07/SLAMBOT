"""
Unit tests for waypoint_navigator helper functions.
Tests pure logic — no ROS2 runtime required.
"""

import math
import unittest
import sys
import os

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'slam_nav_robot'))


def yaw_to_quaternion(yaw_deg: float):
    """Replicated from waypoint_navigator for isolated testing."""
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TestYawToQuaternion(unittest.TestCase):

    def test_zero_yaw(self):
        qz, qw = yaw_to_quaternion(0.0)
        self.assertAlmostEqual(qz, 0.0, places=6)
        self.assertAlmostEqual(qw, 1.0, places=6)

    def test_90_degrees(self):
        qz, qw = yaw_to_quaternion(90.0)
        expected_qz = math.sin(math.pi / 4)
        expected_qw = math.cos(math.pi / 4)
        self.assertAlmostEqual(qz, expected_qz, places=6)
        self.assertAlmostEqual(qw, expected_qw, places=6)

    def test_180_degrees(self):
        qz, qw = yaw_to_quaternion(180.0)
        self.assertAlmostEqual(qz, 1.0, places=6)
        self.assertAlmostEqual(qw, 0.0, places=5)

    def test_270_degrees(self):
        qz, qw = yaw_to_quaternion(270.0)
        expected_qz = math.sin(math.radians(135))
        expected_qw = math.cos(math.radians(135))
        self.assertAlmostEqual(qz, expected_qz, places=6)
        self.assertAlmostEqual(qw, expected_qw, places=6)

    def test_negative_yaw(self):
        qz, qw = yaw_to_quaternion(-90.0)
        self.assertAlmostEqual(qz, -math.sin(math.pi / 4), places=6)
        self.assertAlmostEqual(qw, math.cos(math.pi / 4), places=6)

    def test_unit_quaternion_norm(self):
        """qz² + qw² must equal 1 for any yaw."""
        for deg in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
            qz, qw = yaw_to_quaternion(float(deg))
            norm = qz ** 2 + qw ** 2
            self.assertAlmostEqual(
                norm, 1.0, places=6,
                msg=f'Quaternion not normalised at {deg}°'
            )


class TestWaypointList(unittest.TestCase):

    WAYPOINTS = [
        (1.5, 1.5, 0.0),
        (2.0, -1.5, 90.0),
        (-1.5, -1.0, 180.0),
        (-2.0, -2.0, 270.0),
    ]

    def test_waypoint_count(self):
        self.assertEqual(len(self.WAYPOINTS), 4)

    def test_all_waypoints_within_room_bounds(self):
        """All waypoints must be inside the 8x8m room.world."""
        for x, y, _ in self.WAYPOINTS:
            self.assertGreater(x, -4.0, f'x={x} outside west wall')
            self.assertLess(x, 4.0, f'x={x} outside east wall')
            self.assertGreater(y, -4.0, f'y={y} outside south wall')
            self.assertLess(y, 4.0, f'y={y} outside north wall')

    def test_waypoint_yaw_valid_range(self):
        """Yaw must be between -360 and 360 degrees."""
        for _, _, yaw in self.WAYPOINTS:
            self.assertGreaterEqual(yaw, -360.0)
            self.assertLessEqual(yaw, 360.0)


if __name__ == '__main__':
    unittest.main()
