# ROS2 Robotics Portfolio — `slam_nav_ws`

[![Build and verify](https://github.com/HARSH-2002-07/slam_nav_ws/actions/workflows/build.yml/badge.svg)](https://github.com/HARSH-2002-07/slam_nav_ws/actions/workflows/build.yml)

Two self-contained ROS2 Humble projects sharing a single colcon workspace:

1. **Project 1 — SLAM Navigation Stack** *(complete)* — a single differential-drive robot maps an unknown room with `slam_toolbox`, saves the map, then navigates autonomously through 4 waypoints using Nav2.
2. **Project 2 — Multi-Robot Coordination System** *(complete)* — two namespaced robots share the same map, each runs its own Nav2 lifecycle stack, and a central Python coordinator dispatches tasks nearest-robot-first over `NavigateToPose` action clients.
3. **Project 3 — Vision-Language Robot Controller** *(planned)* — single-robot controller driven by a multimodal LLM (Ollama/LLaVA) turning natural-language commands into `NavigateToPose` goals.

---

## Stack

| Tool | Purpose |
|---|---|
| ROS2 Humble | Middleware |
| Gazebo Classic 11.10 | Simulation |
| SLAM Toolbox | Online mapping (Project 1) |
| Nav2 | Autonomous navigation (both projects) |
| RViz2 | Visualisation |
| Docker + Compose | Reproducible stack (both projects) |
| GitHub Actions | CI: builds both images, verifies entry points and interfaces |

---

## Project 2 — Multi-Robot Coordinator

Two robots (`/robot_1`, `/robot_2`) spawn into the Project 1 map with fully isolated topics, TFs, and Nav2 lifecycle stacks. A Python coordinator subscribes to each `/amcl_pose`, accepts tasks via YAML at startup or the `/fleet_coordinator/send_goal` service, and dispatches them nearest-first using per-robot action clients on `/{robot}/navigate_to_pose`. A companion metrics node aggregates per-robot distance, task counts, and mean time-to-completion from `/odom` + `/fleet_coordinator/status`.

Both robots navigate concurrently on a `MultiThreadedExecutor` with `ReentrantCallbackGroup`. Failures re-queue once, then drop. Everything runs under `docker compose up` — no manual initial-pose step.

### Quick start (Docker)

```bash
docker compose -f docker/docker-compose.yml up --build
# watch the coordinator dispatch tasks:
docker compose -f docker/docker-compose.yml logs -f coordinator
# watch the metrics roll up:
docker compose -f docker/docker-compose.yml logs -f metrics
```

Four services spin up on `network_mode: host`:

| Service | Role |
|---|---|
| `sim` | Gazebo (`gzserver`, headless) + both robot spawns + both Nav2 lifecycle stacks |
| `poses` | One-shot: waits for `/{robot}/amcl` and `/{robot}/bt_navigator` to reach `active`, then publishes both `initialpose` messages |
| `coordinator` | Fleet coordinator node — depends on `poses` completing successfully |
| `metrics` | Fleet metrics node — depends on `poses` completing successfully |

### Quick start (native, 5 terminals)

```bash
colcon build --packages-select multi_robot_coordinator_interfaces
source install/setup.bash
colcon build --packages-select multi_robot_coordinator
source install/setup.bash

# T1 — sim + both Nav2 stacks (wait for both "Managed nodes are active")
ros2 launch multi_robot_coordinator multi_robot.launch.py \
  map_yaml:=$HOME/slam_nav_ws/maps/map_20260410_165455.yaml

# T2 — publish both /initialpose messages (see CLAUDE.md for exact commands)

# T3 — metrics (launch BEFORE coordinator so it sees idle→busy edges)
ros2 launch multi_robot_coordinator fleet_metrics.launch.py

# T4 — coordinator (dispatches the 4 tasks from config/tasks.yaml)
ros2 launch multi_robot_coordinator fleet_coordinator.launch.py

# T5 — add a runtime task
ros2 service call /fleet_coordinator/send_goal \
  multi_robot_coordinator_interfaces/srv/SendGoal \
  "{x: 1.0, y: 0.5, yaw: 0.0}"
```

### Interfaces

```
# SendGoal.srv
float64 x
float64 y
float64 yaw           # radians about Z
---
bool    success
string  task_id       # e.g. "task_0003"

# FleetStatus.msg (publishes on /fleet_coordinator/status)
std_msgs/Header header
string[] robot_names
string[] robot_states   # "idle" | "busy" | "error"
int32    queue_size

# FleetMetrics.msg (publishes on /fleet_metrics/summary at 1 Hz)
std_msgs/Header header
string[]  robot_names
float64[] distance_travelled_m
int32[]   tasks_completed
float64   mean_seconds_per_task
int32     total_tasks_dispatched
```

### Tests

Five pytest + rclpy integration tests (no Gazebo/Nav2 required — mocks the `NavigateToPose` servers in-process):

```bash
python3 -m pytest src/multi_robot_coordinator/test/test_fleet_coordinator.py -v
```

Covers: service wiring, happy-path drain, two-robot parallelism, abort → retry-once → drop, both-busy queue waiting.

---

## Project 1 — SLAM Navigation Stack *(archived, still in tree)*

Single differential-drive robot. SLAM Toolbox runs in online mode while the user drives the robot around `room.world` with teleop, then the map is saved to `maps/`. A second launch brings up Nav2 (AMCL localisation, NavFn planner, DWB controller), and a waypoint runner sends the robot through 4 autonomous goals.

Docker + CI for Project 1 live at the repo root (`Dockerfile`, `docker-compose.yml`).

---

## Repository Layout

```
slam_nav_ws/
├── src/
│   ├── slam_nav_robot/                         # Project 1 package
│   ├── multi_robot_coordinator/                # Project 2 main package
│   └── multi_robot_coordinator_interfaces/     # Project 2 custom msgs/srvs
├── docker/                                     # Project 2 Docker stack
│   ├── Dockerfile.multi_robot
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── wait_and_pose.sh
├── Dockerfile                                  # Project 1 image
├── docker-compose.yml                          # Project 1 stack
├── maps/                                       # saved SLAM maps (reused by Project 2)
├── .github/workflows/build.yml                 # 2 CI jobs: build, build_multi_robot
├── CLAUDE.md                                   # context handoff for Claude Code
└── README.md                                   # this file
```

---

## CI

GitHub Actions builds both Docker images on every push to `main` and every PR. Two independent jobs:

- `build` — `Dockerfile` (Project 1); verifies colcon install sources and entry points are discoverable
- `build_multi_robot` — `docker/Dockerfile.multi_robot` (Project 2); additionally verifies all three custom interfaces (`SendGoal`, `FleetStatus`, `FleetMetrics`) are registered and `fleet_coordinator` / `fleet_metrics` entry points exist

Separate buildx caches per job keep either from invalidating the other.

---

## Author

**Harsh Jain** · [GitHub](https://github.com/HARSH-2002-07) · Building a 3-month robotics portfolio targeting EU robotics roles and EPFL/ETH master's applications.
