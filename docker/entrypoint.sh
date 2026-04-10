#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Pass through any command given to docker run
exec "$@"