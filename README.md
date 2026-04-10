# ROS2 SLAM Navigation Stack

Autonomous mapping and navigation for a simulated differential-drive robot,
built entirely in ROS2 Humble + Gazebo Classic. Week 1 of a 4-week project.

## Stack

| Tool | Purpose |
|---|---|
| ROS2 Humble | Middleware |
| Gazebo Classic 11 | Simulation |
| SLAM Toolbox | Mapping |
| Nav2 | Autonomous navigation |
| RViz2 | Visualisation |

## Quick Start

```bash
# 1. Build
cd ~/slam_nav_ws
colcon build --symlink-install
source install/setup.bash

# 2. Launch simulation
ros2 launch slam_nav_robot gazebo.launch.py

# 3. Open RViz2 (new terminal)
source ~/slam_nav_ws/install/setup.bash
ros2 launch slam_nav_robot display.launch.py

# 4. Drive the robot (new terminal)
source ~/slam_nav_ws/install/setup.bash
ros2 run slam_nav_robot teleop_node
```

## Results

*(Screenshots and demo video link will go here after Week 2)*

## Author

Harsh Jain — [GitHub](https://github.com/YOUR_USERNAME) · [LinkedIn](https://linkedin.com/in/YOUR_USERNAME)