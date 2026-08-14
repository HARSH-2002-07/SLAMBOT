# CLAUDE.md — Multi-Robot Coordination System
## Context Handoff for Claude Code

> **Read this entire file before touching any code.**
> This is a living context document. Update it as you complete tasks.

---

## 🚦 CURRENT STATUS (read me first)

**Last updated:** 2026-04-19
**Active phase:** **Project 2 closed out — next up is Project 3**

| Phase | Status | Verification |
|---|---|---|
| Week 5 — Multi-robot sim (Gazebo, namespaced) | ✅ DONE | Both robots spawn; `/scan`, `/cmd_vel`, TF isolated |
| Week 6 — Per-robot Nav2 stacks | ✅ DONE | Both lifecycle managers active; 7 bonded nodes each |
| Week 7 — Fleet coordinator node | ✅ DONE | 4/4 YAML tasks dispatched in parallel, all SUCCEEDED |
| **Week 8 — Docker / tests / metrics / demo** | **✅ DONE** | 5/5 integration tests green; GitHub CI builds both images; video recorded |

**What works end-to-end right now:**
- **Native path (5 terminals):** `multi_robot.launch.py` → publish 2 `initialpose` messages → `fleet_metrics.launch.py` (must come BEFORE coordinator so it sees the idle→busy edges) → `fleet_coordinator.launch.py` → tasks from `tasks.yaml` are dispatched nearest-first, both robots navigate concurrently, `/fleet_coordinator/status` reports state, `/fleet_metrics/summary` reports distance + tasks completed + mean time per task.
- **Dockerised path (one command):** `docker compose -f docker/docker-compose.yml up --build` — compose brings up Gazebo + both Nav2 stacks + a one-shot pose publisher + coordinator + metrics, all on `network_mode: host`. Coordinator/metrics wait for `poses` via `service_completed_successfully`, so no manual steps.

**What's still manual (by design):** the native path keeps the two `ros2 topic pub /initialpose` calls manual — it's useful for debugging and matches how you'd run it at a workstation. Docker Compose is the automated version.

---

## Who I Am & What I'm Building

I'm building a **3-month robotics portfolio** targeting EU robotics jobs and EPFL/ETH master's applications. The stack is ROS2 Humble + Python, running on Ubuntu with Gazebo Classic (11.10.2).

**Portfolio structure:**
- ✅ Project 1 — ROS2 SLAM Nav Stack *(complete, submitted)*
- ✅ Project 2 — Multi-Robot Coordination System *(all four weeks complete)*
- ⬜ Project 3 — Vision-Language Robot Controller (ROS2 + LLaVA/Ollama + OpenCV)

---

## Workspace Layout

```
~/slam_nav_ws/
├── src/
│   ├── multi_robot_coordinator/                    ← main ROS2 package (Python)
│   │   ├── multi_robot_coordinator/
│   │   │   ├── fleet_coordinator.py                ← Week 7 coordinator node
│   │   │   └── fleet_metrics.py                    ← Week 8 metrics node
│   │   ├── launch/
│   │   │   ├── multi_robot.launch.py               ← spawns both robots + Nav2 stacks (headless arg)
│   │   │   ├── nav2_robot.launch.py                ← single-robot Nav2 launcher (reusable)
│   │   │   ├── spawn_robot.launch.py               ← single-robot spawner
│   │   │   ├── fleet_coordinator.launch.py         ← launches the coordinator only
│   │   │   └── fleet_metrics.launch.py             ← launches the metrics only
│   │   ├── config/
│   │   │   ├── nav2_params.yaml                    ← Nav2 params (RewrittenYaml per robot)
│   │   │   └── tasks.yaml                          ← task list for fleet coordinator
│   │   ├── description/
│   │   │   └── robot.urdf.xacro                    ← parameterised URDF
│   │   ├── worlds/
│   │   │   └── room.world
│   │   ├── test/
│   │   │   └── test_fleet_coordinator.py           ← 5 pytest+rclpy integration tests
│   │   ├── package.xml
│   │   └── setup.py
│   └── multi_robot_coordinator_interfaces/         ← CMake pkg (interfaces only)
│       ├── srv/SendGoal.srv
│       ├── msg/FleetStatus.msg
│       ├── msg/FleetMetrics.msg
│       ├── CMakeLists.txt
│       └── package.xml
├── docker/                                         ← Project 2 Docker stack
│   ├── Dockerfile.multi_robot                      ← multi-stage build for coordinator + interfaces
│   ├── docker-compose.yml                          ← sim + poses + coordinator + metrics
│   ├── entrypoint.sh                               ← sources ROS2 + workspace
│   └── wait_and_pose.sh                            ← waits for AMCL active, publishes both initial poses
├── Dockerfile                                      ← Project 1 Docker (unchanged)
├── docker-compose.yml                              ← Project 1 Docker stack (unchanged)
├── maps/
│   └── map_20260410_165455.yaml                    ← map from Project 1 SLAM run
│       map_20260410_165455.pgm
├── .github/workflows/build.yml                     ← 2 jobs: build + build_multi_robot
└── install/   build/   log/                        ← colcon artefacts (never edit)
```

---

## Project 2 — Current State

### ✅ Week 5 — Multi-Robot Simulation (COMPLETE)

**What was built:**
- Namespace architecture: `/robot_1` and `/robot_2` — fully isolated topics, TFs, params
- `robot.urdf.xacro` — parameterised with `robot_name` and `robot_ns` xacro args
- `spawn_robot.launch.py` — reusable single-robot spawner (pose + namespace args)
- `multi_robot.launch.py` — spawns both robots with a 3s `TimerAction` delay between them

**Verified working:**
- Both robots spawn in Gazebo with independent TF chains (`robot_1/base_footprint`, `robot_2/base_footprint`)
- `/robot_1/scan` and `/robot_2/scan` publishing at 10 Hz
- `/robot_1/cmd_vel` and `/robot_2/cmd_vel` driving their respective diff_drive plugins

**Key bugs that were fixed (do NOT reintroduce):**
- `<xacro:arg>` inside plugin tags outputs nothing — always use `$(arg name)` for substitution
- Leading slash on `robot_ns` causes double-slash topics (`//robot_1/scan`) — pass without leading slash
- `robot_description` param must be wrapped in `ParameterValue(value_type=str)` in launch files
- `data_files` in `setup.py` must explicitly list every subdirectory (`description/`, `launch/`, `config/`, `worlds/`)

---

### ✅ Week 6 — Per-Robot Nav2 Stacks (COMPLETE)

**What was built:**
- `nav2_params.yaml` — single template file; `RewrittenYaml` injects per-robot frame names at launch time
- `nav2_robot.launch.py` — reusable single-robot Nav2 launcher using `PushRosNamespace` + `GroupAction`
- Updated `multi_robot.launch.py` — Nav2 stacks start delayed (8s robot_1, 10s robot_2) after Gazebo stabilises

**Verified working:**
- Both lifecycle managers reach `Managed nodes are active` with all 7 nodes bonded per robot:
  `map_server`, `amcl`, `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`
- `behavior_server` loads 4 plugins: `spin`, `backup`, `drive_on_heading`, `wait` (all from `nav2_behaviors`)
- `/robot_1/amcl_pose` and `/robot_2/amcl_pose` publish after initial pose is set
- `/robot_1/navigate_to_pose` and `/robot_2/navigate_to_pose` action servers are live

**Key bugs that were fixed (do NOT reintroduce):**
- Package renamed: `nav2_recoveries` → `nav2_behaviors` (Humble breaking change)
- Key renamed: `recovery_plugins` → `behavior_plugins` in nav2_params.yaml
- Plugin paths changed: `nav2_recoveries/Spin` → `nav2_behaviors/Spin` (same for BackUp, DriveOnHeading, Wait)
- Lifecycle manager's `node_names` list: `recoveries_server` → `behavior_server`

**Native path initial-pose publish (still required for the non-Docker workflow):**
```bash
ros2 topic pub --once /robot_1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: -0.879923, y: -0.024577, z: 0.019997},
      orientation: {x: 0.0, y: 0.0, z: -0.3229, w: 0.9464}
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]
  }
}' --ros-args -p use_sim_time:=true

ros2 topic pub --once /robot_2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: 1.028013, y: -0.385103, z: 0.019999},
      orientation: {x: 0.0, y: 0.0, z: 0.99335193, w: 0.11511528}
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]
  }
}'
```
These coordinates match the actual Gazebo spawn positions in the `room.world`. The Docker `wait_and_pose.sh` automates the same calls.

---

### ✅ Week 7 — Fleet Coordinator Node (COMPLETE)

**What was built:**
- `multi_robot_coordinator_interfaces/` — new CMake package with `srv/SendGoal.srv` and `msg/FleetStatus.msg`
- `fleet_coordinator.py` — nearest-idle dispatcher using `MultiThreadedExecutor` + `ReentrantCallbackGroup`, thread-safe task queue, per-robot action clients for `/{robot}/navigate_to_pose`, AMCL pose subscribers, runtime `SendGoal` service, `FleetStatus` publisher at 1 Hz, dispatch timer at 2 Hz, `max_retries=1` with re-queue on failure
- `fleet_coordinator.launch.py` — launches the coordinator only (Nav2 is expected to be running already)
- `config/tasks.yaml` — sample 4-task list
- Registered `fleet_coordinator` entry point in `setup.py`; added `nav2_msgs`, `std_msgs`, `action_msgs`, `multi_robot_coordinator_interfaces`, `python3-yaml` deps in `package.xml`

**Verified working:**
- Cold start with 4 YAML tasks → both robots dispatched in parallel → all 4 tasks `SUCCEEDED`
- Second task dispatched the moment the first completes (state flips `busy`→`idle` in the result callback)
- Runtime `SendGoal` service accepts goals and returns a `task_id`
- `/fleet_coordinator/status` publishes `FleetStatus` with `robot_states` and `queue_size`

**Design decisions (locked in):**
| Decision | Choice | Reason |
|---|---|---|
| Task assignment | Nearest-robot-first | More realistic for fleet demo |
| Task input | YAML at startup + `/fleet_coordinator/send_goal` service | Flexible for demo and testing |
| Failure handling | Re-queue once (max_retries=1), then drop with ERROR | Safe default |
| Executor | `MultiThreadedExecutor` + `ReentrantCallbackGroup` | Both robots navigate in parallel |

**Interfaces (final):**

`SendGoal.srv`
```
float64 x
float64 y
float64 yaw        # radians around Z
---
bool   success
string task_id     # e.g. "task_0003"
```

`FleetStatus.msg`
```
std_msgs/Header header
string[] robot_names
string[] robot_states
int32    queue_size
```

**Key bugs that were fixed (do NOT reintroduce):**
- `bt_navigator` rejected every goal when `default_nav_to_pose_bt_xml` was `""` — must point at a real BT XML file (e.g. `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`)
- Stock BT XMLs reference plugins that must be present in `bt_navigator.plugin_lib_names`: add `nav2_remove_passed_goals_action_bt_node`, `nav2_goal_updater_node_bt_node`, `nav2_drive_on_heading_bt_node`, `nav2_globally_updated_goal_condition_bt_node`
- `nav2_drive_on_heading_action_bt_node` does NOT exist in Humble — the correct lib is `nav2_drive_on_heading_bt_node` (no `_action` infix)
- Build order: always `colcon build --packages-select multi_robot_coordinator_interfaces` before the main package
- Initial pose for both robots must be published **before** launching the coordinator, otherwise `/amcl_pose` is silent and dispatch falls back to "any idle robot"

**Known minor issue (not blocking):**
`_pick_nearest` skips robots whose `/amcl_pose` hasn't arrived yet. At cold start the first task may go to whichever robot published first, not the geometrically nearest one. Fallback exists only when *all* poses are unknown. Harmless for the demo; can be hardened later.

---

### ✅ Week 8 — Docker + Tests + Metrics + Demo (COMPLETE)

**What was built:**

**Fleet metrics node** — `fleet_metrics.py`
- Subscribes `/{robot}/odom` for per-robot distance integration, `/fleet_coordinator/status` for task state transitions
- Publishes `FleetMetrics.msg` on `/fleet_metrics/summary` at 1 Hz
- Tracks per-robot distance (m), tasks completed (busy→idle edges), mean seconds per task, total tasks dispatched
- Notes in docstring: counts busy phases, so an abort+retry counts as 2 completions. Distance derives from `/odom` (smooth, high-rate) not AMCL (jumpy)

**Coordinator event-driven status publishing** — edit to `fleet_coordinator.py`
- `threading.Lock` → `threading.RLock` so `_publish_status()` can be called from inside a locked section
- Added `self._publish_status()` calls immediately after every state change in `_dispatch_tick`, `_result_cb` success branch, and `_fail_or_requeue`
- Reason: the 1 Hz heartbeat was too slow for the metrics node to reliably see the idle→busy→idle edges when tasks completed faster than 1 s. Event-driven publication closes that gap

**Integration tests** — `test/test_fleet_coordinator.py` (5 tests, ~13 s total)
- pytest + rclpy + `MultiThreadedExecutor`; mocks `NavigateToPose` action servers so no Gazebo/Nav2 needed
- Module-scoped mocks + per-test coordinator fixture to avoid DDS discovery races
- Covers: service wiring, happy-path drain, two-robot parallelism, abort→retry-once→drop, both-busy-queue-waits

**Multi-robot Docker Compose** — `docker/` dir
- `Dockerfile.multi_robot` — multi-stage (base → builder → runtime), interfaces built before main package
- `docker-compose.yml` — 4 services on `network_mode: host`: `sim` (Gazebo headless + both Nav2), `poses` (one-shot wait + publish), `coordinator`, `metrics`. `coordinator`/`metrics` gate on `poses: service_completed_successfully`
- `wait_and_pose.sh` — polls `ros2 lifecycle get /{robot}/amcl` and `/{robot}/bt_navigator` for `active`, then publishes both initial poses, then waits for `/amcl_pose` to come alive
- `headless:=true` added as a launch arg on `multi_robot.launch.py` — swaps `gazebo` for `gzserver` when running in Docker

**GitHub Actions CI** — `build.yml`
- Existing `build` job unchanged (builds Project 1 image)
- New `build_multi_robot` job builds `docker/Dockerfile.multi_robot`, then verifies entry points (`fleet_coordinator`, `fleet_metrics`) and registers all three interfaces (`SendGoal`, `FleetStatus`, `FleetMetrics`)
- Separate buildx cache so the two jobs don't invalidate each other

**Demo video** — 60–90 s recorded, showing Docker stack boot → parallel dispatch → metrics output → runtime `send_goal` service call

**Key bugs that were fixed (do NOT reintroduce):**
- Metrics counted zero idle→busy→idle transitions because coordinator only heartbeated status at 1 Hz; fix is to publish on every state change (event-driven, not poll)
- Early pytest runs picked up a still-running Nav2 stack from a prior native session and received each goal on two action servers. Tests require no live `/fleet_coordinator` / `/robot_*/navigate_to_pose` nodes before starting — best run on a clean graph or after `pkill -f ros2`
- Between tests, DDS needs a moment to tear down old action servers — use module-scoped mocks and a per-test FleetCoordinator fixture (reset mock counters between tests, don't recreate the ActionServer)
- Pytest warns "cannot collect test class 'TestHarness'" because pytest treats `Test*` classes as collection candidates — rename helper to `Harness` (or similar)
- Project 1's Dockerfile does `COPY src/ src/` + unconstrained `colcon build`, so adding new packages to src/ could break its CI job. Kept working because rosdep resolves the new deps transitively; but if you add another package, verify the first CI job still passes

**Design decisions (locked in):**
| Decision | Choice | Reason |
|---|---|---|
| Metrics transport | Status-transition edge-detection, not action subscription | Coordinator owns truth; metrics stays passive |
| Retry semantics in metrics | Count every busy phase as a task completion | Keep metrics node independent; documented in docstring |
| Test isolation | Mock NavigateToPose servers in-process | No Gazebo dependency, fast (~13 s for 5 tests) |
| Docker startup gate | `depends_on: service_completed_successfully` | Deterministic, no timed sleeps |
| Gazebo in Docker | `gzserver` headless via `headless:=true` launch arg | No X11 forwarding needed for CI/video |
| CI strategy | Two independent jobs, separate caches | Either project can fail without blocking the other |

**How to run (Docker):**
```bash
cd ~/slam_nav_ws
docker compose -f docker/docker-compose.yml up --build
# Tail coordinator:
docker compose -f docker/docker-compose.yml logs -f coordinator
# Tail metrics:
docker compose -f docker/docker-compose.yml logs -f metrics
# Clean shutdown:
docker compose -f docker/docker-compose.yml down
```

**How to run tests:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/multi_robot_coordinator/test/test_fleet_coordinator.py -v
# or via colcon:
colcon test --packages-select multi_robot_coordinator --event-handlers console_direct+
```

---

## Project 1 — Reference (already submitted)

- ROS2 SLAM pipeline: `slam_toolbox` online mode → saved map
- Nav2 single-robot: AMCL localisation + NavFn planner + DWB controller
- 4 autonomous waypoints navigated successfully
- Docker image published, CI/CD via GitHub Actions
- Workspace: `~/slam_nav_ws` (same workspace, different package)

---

## Environment

| Item | Value |
|---|---|
| OS | Ubuntu (ROS2 Humble) |
| Simulator | Gazebo Classic 11.10.2 |
| ROS2 distro | Humble Hawksbill |
| Python | 3.10 |
| Workspace | `~/slam_nav_ws` |
| Map file | `~/slam_nav_ws/maps/map_20260410_165455.yaml` |
| use_sim_time | `true` always |

---

## Common Gotchas (Learn From These)

| Gotcha | Fix |
|---|---|
| `xacro:arg` inside Gazebo plugin tags | Use `$(arg name)` not `${arg}` |
| Leading slash on namespace → `//robot_1/scan` | Pass namespace without leading slash |
| `robot_description` param type error | Wrap in `ParameterValue(value_type=str)` |
| `nav2_recoveries` package not found | Renamed to `nav2_behaviors` in Humble |
| Lifecycle manager stuck on `recoveries_server` | Rename to `behavior_server` everywhere |
| `setup.py` missing subdirectory | Explicitly list every dir in `data_files` |
| AMCL warning loop on startup | Normal — just needs initial pose published |
| colcon build order | Always build `_interfaces` package first |
| `bt_navigator` rejects every goal immediately | `default_nav_to_pose_bt_xml` must be a real path, not `""` |
| `bt_navigator` fails to activate with `Node not recognized: <X>` | Missing plugin in `plugin_lib_names` — find the matching `libnav2_*_bt_node.so` in `/opt/ros/humble/lib/` |
| `libnav2_drive_on_heading_action_bt_node.so not found` | It's `nav2_drive_on_heading_bt_node` in Humble (no `_action` infix) |
| Only one robot receives tasks at cold start | `_pick_nearest` skips robots whose `/amcl_pose` hasn't arrived yet — publish initial pose for both before launching the coordinator |
| Metrics shows `tasks_completed=[0,0]` despite tasks running | Coordinator must publish `FleetStatus` on every state change (not only on the 1 Hz heartbeat) so transitions faster than 1 s are visible |
| Tests show "more than one action server for `/robot_*/navigate_to_pose`" | Kill any live Nav2/coordinator processes before running pytest; or run in Docker |
| pytest "cannot collect TestHarness class" warning | Don't prefix helper classes with `Test` — rename to `Harness` |

---

## What To Do Right Now

**Project 2 is done.** Weeks 5–8 all verified end-to-end: native + Docker paths both work, 5/5 integration tests pass, CI is green on both jobs, video recorded. Nothing in Project 2 files should be modified unless there is a clear bug.

**Next phase:** Kick off **Project 3 — Vision-Language Robot Controller**. Planned stack: ROS2 Humble + Ollama (LLaVA or similar multimodal model) + OpenCV. Target scope is a single-robot controller that accepts natural-language commands referencing objects it sees in a camera feed ("drive to the red block", "go to the doorway on the right") and issues `NavigateToPose` goals accordingly. Keep reusing `~/slam_nav_ws` — just a new package under `src/`.

Suggested first discussions before writing code:
1. Decide on model + inference path (local Ollama vs. remote API; latency budget; GPU available?)
2. Camera pipeline in Gazebo (URDF camera plugin? which sensor topics?)
3. Prompting & grounding strategy (bounding-box output from VLM, or natural language → 2D goal transform?)
4. Failure/fallback behaviour when the VLM is uncertain
