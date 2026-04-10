"""
Tests that the robot URDF/XACRO model is valid and contains
all required links and joints.
"""

import os
import subprocess
import unittest

from ament_index_python.packages import get_package_share_directory


class TestURDF(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pkg_share = get_package_share_directory('slam_nav_robot')
        xacro_file = os.path.join(
            pkg_share, 'description', 'robot.urdf.xacro')

        result = subprocess.run(
            ['xacro', xacro_file],
            capture_output=True, text=True
        )
        cls.urdf_xml = result.stdout
        cls.xacro_ok = result.returncode == 0

    def test_xacro_parses_without_error(self):
        """XACRO must expand without any errors."""
        self.assertTrue(
            self.xacro_ok,
            'xacro failed to parse robot.urdf.xacro'
        )

    def test_urdf_has_base_link(self):
        self.assertIn('<link name="base_link"', self.urdf_xml)

    def test_urdf_has_base_footprint(self):
        self.assertIn('<link name="base_footprint"', self.urdf_xml)

    def test_urdf_has_lidar_link(self):
        self.assertIn('<link name="lidar_link"', self.urdf_xml)

    def test_urdf_has_left_wheel(self):
        self.assertIn('<link name="left_wheel"', self.urdf_xml)

    def test_urdf_has_right_wheel(self):
        self.assertIn('<link name="right_wheel"', self.urdf_xml)

    def test_urdf_has_caster_wheel(self):
        self.assertIn('<link name="caster_wheel"', self.urdf_xml)

    def test_urdf_has_diff_drive_plugin(self):
        self.assertIn('libgazebo_ros_diff_drive.so', self.urdf_xml)

    def test_urdf_has_lidar_plugin(self):
        self.assertIn('libgazebo_ros_ray_sensor.so', self.urdf_xml)

    def test_urdf_has_wheel_joints(self):
        self.assertIn('left_wheel_joint', self.urdf_xml)
        self.assertIn('right_wheel_joint', self.urdf_xml)


if __name__ == '__main__':
    unittest.main()
