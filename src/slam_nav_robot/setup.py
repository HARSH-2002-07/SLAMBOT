from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_nav_robot'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'),
            glob('description/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Jain',
    maintainer_email='jainharshdev@gmail.com',
    description='ROS2 SLAM Navigation Stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node   = slam_nav_robot.teleop_node:main',
            'map_saver     = slam_nav_robot.map_saver:main',
            'bag_recorder  = slam_nav_robot.bag_recorder:main',
        ],
    },
)