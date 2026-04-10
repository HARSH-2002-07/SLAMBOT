# ─────────────────────────────────────────────────────────────
# Stage 1: base — ROS2 Humble + system dependencies
# ─────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install all runtime dependencies in one layer
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-twist-mux \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────
# Stage 2: builder — copy source and build workspace
# ─────────────────────────────────────────────────────────────
FROM base AS builder

WORKDIR /ros2_ws

# Copy package manifests first (better Docker layer caching)
COPY src/slam_nav_robot/package.xml src/slam_nav_robot/
COPY src/slam_nav_robot/setup.py    src/slam_nav_robot/
COPY src/slam_nav_robot/setup.cfg   src/slam_nav_robot/

# Install ROS dependencies declared in package.xml
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Copy full source
COPY src/ src/

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build/

# ─────────────────────────────────────────────────────────────
# Stage 3: runtime — lean final image
# ─────────────────────────────────────────────────────────────
FROM base AS runtime

WORKDIR /ros2_ws

# Copy only the built install directory from builder
COPY --from=builder /ros2_ws/install ./install

# Source ROS2 + workspace on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Entrypoint sources both setups before any command
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]