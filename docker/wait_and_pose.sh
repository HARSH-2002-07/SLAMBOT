#!/bin/bash
# Wait for both robots' Nav2 stacks to be active, then publish initial poses.
#
# Runs inside the multi_robot_coordinator container. Exits 0 after both
# initial poses have been published and AMCL has produced /amcl_pose.
#
# Intended as a one-shot compose service that unblocks the coordinator.

set -eu

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ROBOTS=("robot_1" "robot_2")
MAX_WAIT=${MAX_WAIT:-180}        # seconds total budget
POLL_INTERVAL=2

wait_for_lifecycle_active() {
    local node="$1"
    local deadline=$(( $(date +%s) + MAX_WAIT ))
    echo "[wait_and_pose] waiting for ${node} to reach lifecycle 'active'..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if ros2 lifecycle get "$node" 2>/dev/null | grep -q "active"; then
            echo "[wait_and_pose] ${node} is active."
            return 0
        fi
        sleep "$POLL_INTERVAL"
    done
    echo "[wait_and_pose] TIMEOUT: ${node} never reached 'active'." >&2
    return 1
}

publish_pose() {
    local ns="$1"; local x="$2"; local y="$3"; local qz="$4"; local qw="$5"
    echo "[wait_and_pose] publishing /${ns}/initialpose -> (${x}, ${y})"
    ros2 topic pub --once "/${ns}/initialpose" \
        geometry_msgs/msg/PoseWithCovarianceStamped "{
            header: {frame_id: 'map'},
            pose: {
              pose: {
                position: {x: ${x}, y: ${y}, z: 0.02},
                orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}
              },
              covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                           0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]
            }
        }" --ros-args -p use_sim_time:=true >/dev/null
}

wait_for_amcl_pose() {
    local ns="$1"
    local deadline=$(( $(date +%s) + 60 ))
    echo "[wait_and_pose] waiting for /${ns}/amcl_pose to publish..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        # `ros2 topic echo --once` blocks until one message; bail out if timeout.
        if timeout 5 ros2 topic echo --once "/${ns}/amcl_pose" \
                geometry_msgs/msg/PoseWithCovarianceStamped >/dev/null 2>&1; then
            echo "[wait_and_pose] /${ns}/amcl_pose is live."
            return 0
        fi
        sleep "$POLL_INTERVAL"
    done
    echo "[wait_and_pose] TIMEOUT: /${ns}/amcl_pose never published." >&2
    return 1
}

for r in "${ROBOTS[@]}"; do
    wait_for_lifecycle_active "/${r}/amcl"
    wait_for_lifecycle_active "/${r}/bt_navigator"
done

# Exact spawn poses used in multi_robot.launch.py / room.world.
# robot_1 spawns at (-1, 0); robot_2 at (1, 0). Initial pose uses the
# coordinates actually observed in Gazebo (see CLAUDE.md Week 6 block).
publish_pose "robot_1" "-0.879923"  "-0.024577"  "-0.3229"        "0.9464"
publish_pose "robot_2"  "1.028013"  "-0.385103"   "0.99335193"    "0.11511528"

for r in "${ROBOTS[@]}"; do
    wait_for_amcl_pose "$r"
done

echo "[wait_and_pose] both robots localised; coordinator can start."
