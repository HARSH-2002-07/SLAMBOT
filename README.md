# ROS2 Robotics Portfolio — `slam_nav_ws`

[![Build and verify](https://github.com/HARSH-2002-07/slam_nav_ws/actions/workflows/build.yml/badge.svg)](https://github.com/HARSH-2002-07/slam_nav_ws/actions/workflows/build.yml)

Three self-contained ROS2 Humble projects sharing a single colcon workspace:

1. **Project 1 — SLAM Navigation Stack** *(complete)* — a single differential-drive robot maps an unknown room with `slam_toolbox`, saves the map, then navigates autonomously through 4 waypoints using Nav2.
2. **Project 2 — Multi-Robot Coordination System** *(complete)* — two namespaced robots share the same map, each runs its own Nav2 lifecycle stack, and a central Python coordinator dispatches tasks nearest-robot-first over `NavigateToPose` action clients.
3. **Project 3 — Vision-Language Robot Controller** *(complete)* — single-robot controller that turns natural-language target descriptions ("red box", "blue cylinder") into `NavigateToPose` goals, grounded by Google Gemini 2.5 Flash on live RGB-D camera frames.

---

## Stack

| Tool | Purpose |
|---|---|
| ROS2 Humble | Middleware |
| Gazebo Classic 11.10 | Simulation |
| SLAM Toolbox | Online mapping (Project 1) |
| Nav2 | Autonomous navigation (all three projects) |
| Google Gemini 2.5 Flash | Vision-language object grounding (Project 3) |
| RViz2 | Visualisation |
| Docker + Compose | Reproducible stack (Projects 1 & 2) |
| GitHub Actions | CI: builds all images, verifies entry points and interfaces |

---

## Project 3 — Vision-Language Robot Controller

A robot inside Gazebo accepts a natural-language target ("red box", "blue cylinder") over a ROS 2 service. The pipeline:

1. Grabs the latest RGB + depth frame + `camera_info`
2. Sends the RGB frame to **Gemini 2.5 Flash** with a schema-validated JSON prompt asking for a 2D bounding box around the named object
3. Back-projects the bbox centre pixel through the depth frame into a 3D point in the camera's optical frame (upper-60%-of-bbox depth sampling avoids floor leakage on ground-sitting objects)
4. TF2-transforms that point into the `map` frame
5. Computes a standoff goal 0.3 m short of the target so Nav2 doesn't plan into the object's surface
6. Sends a `NavigateToPose` action goal and returns the computed `PoseStamped` via the service

Full pipeline detail, parameter reference, and troubleshooting log: [`docs/vlm_nav_robot.md`](docs/vlm_nav_robot.md).

### Quick start (native, 4 terminals)

```bash
colcon build --packages-select vlm_nav_robot_interfaces
source install/setup.bash
colcon build --packages-select vlm_nav_robot --symlink-install
source install/setup.bash
export GEMINI_API_KEY="your-key"

# T1 — full stack (Gazebo + Nav2 + grounder)
ros2 launch vlm_nav_robot vlm_full.launch.py

# T2 — after "Managed nodes are active", set initial pose (see docs/vlm_nav_robot.md §9)

# T3 — RViz2 (optional), Fixed Frame: map

# T4 — call the service
ros2 service call /vlm_grounder/find_and_go \
  vlm_nav_robot_interfaces/srv/FindAndGo "{target: 'red box'}"
```

### Interfaces

```
# FindAndGo.srv
string target
---
bool   success
string reason
geometry_msgs/PoseStamped goal
```

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

## Project 1 — SLAM Navigation Stack

Single differential-drive robot. SLAM Toolbox runs in online mode while the user drives the robot around `room.world` with teleop, then the map is saved to `maps/`. A second launch brings up Nav2 (AMCL localisation, NavFn planner, DWB controller), and a waypoint runner sends the robot through 4 autonomous goals.

Docker + CI for Project 1 live at the repo root (`Dockerfile`, `docker-compose.yml`).

### Quick start (Docker)

```bash
docker compose up --build
```

### Tests

```bash
python3 -m pytest src/slam_nav_robot/test/ -v
```

---

## Repository Layout

```
slam_nav_ws/
├── src/
│   ├── slam_nav_robot/                         # Project 1 package
│   ├── multi_robot_coordinator/                # Project 2 main package
│   ├── multi_robot_coordinator_interfaces/     # Project 2 custom msgs/srvs
│   ├── vlm_nav_robot/                          # Project 3 main package
│   └── vlm_nav_robot_interfaces/               # Project 3 custom srv (FindAndGo)
├── docker/                                     # Project 2 Docker stack
│   ├── Dockerfile.multi_robot
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── wait_and_pose.sh
├── docs/
│   └── vlm_nav_robot.md                        # Project 3 full reference doc
├── Dockerfile                                  # Project 1 image
├── docker-compose.yml                          # Project 1 Docker stack (root)
├── maps/                                       # saved SLAM maps (reused by Projects 2 & 3)
├── .github/workflows/build.yml                 # CI: build, build_multi_robot (+ build_vlm_robot pending)
├── CLAUDE.md                                   # project memory / context handoff
└── README.md                                   # this file
```

---

## CI

GitHub Actions builds Docker images on every push to `main` and every PR.

- `build` — `Dockerfile` (Project 1); verifies colcon install sources and entry points are discoverable
- `build_multi_robot` — `docker/Dockerfile.multi_robot` (Project 2); additionally verifies all three custom interfaces (`SendGoal`, `FleetStatus`, `FleetMetrics`) are registered and `fleet_coordinator` / `fleet_metrics` entry points exist
- `build_vlm_robot` *(pending)* — will build `vlm_nav_robot` + `vlm_nav_robot_interfaces` and verify the `FindAndGo` interface and `vlm_grounder` entry point, mirroring the two jobs above

Separate buildx caches per job keep any one job from invalidating the others.

---

## Author

**Harsh Jain** · [GitHub](https://github.com/HARSH-2002-07) · Building a 3-month robotics portfolio targeting EU robotics roles and EPFL/ETH master's applications.