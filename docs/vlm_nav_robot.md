# Project 3 — Vision-Language Robot Controller
## Complete Reference Document

**Stack:** ROS 2 Humble · Gazebo Classic 11 · Google Gemini 2.5 Flash · Nav2 · Python 3.10  
**Workspace:** `~/slam_nav_ws`  
**Package:** `vlm_nav_robot` + `vlm_nav_robot_interfaces`  
**Status:** Week 10 Part B complete — full pipeline working end-to-end

---

## 1. What This Project Does

A robot inside a Gazebo simulation can be sent natural-language commands like `"red box"` or `"blue cylinder"`. The system:

1. Grabs the current RGB frame and depth frame from the robot's RGB-D camera
2. Sends the image to **Gemini 2.5 Flash** with a structured prompt asking for a bounding box around the named object
3. Uses the bbox + depth to back-project the target into 3D space (camera optical frame)
4. Transforms the 3D point into the `map` frame via TF2
5. Computes a **standoff goal** 0.3 m short of the target so Nav2 doesn't plan into the object surface
6. Sends a `NavigateToPose` action goal to Nav2 — the robot drives autonomously to the target
7. Returns the computed `PoseStamped` via a ROS 2 service so the caller can log or chain goals

---

## 2. Repository Layout

```
~/slam_nav_ws/
├── src/
│   ├── vlm_nav_robot/                          ← main ROS 2 package (Python)
│   │   ├── vlm_nav_robot/
│   │   │   └── vlm_grounder.py                 ← core node: Gemini + depth + Nav2
│   │   ├── launch/
│   │   │   ├── vlm_full.launch.py              ← Week 10 full stack (Gazebo + Nav2 + grounder)
│   │   │   └── vlm_demo.launch.py              ← Week 9 sim-only (no Nav2, no grounder)
│   │   ├── config/
│   │   │   └── nav2_params.yaml                ← Nav2 params for single robot
│   │   ├── description/
│   │   │   └── robot.urdf.xacro                ← diff-drive robot + 2D LiDAR + RGB-D camera
│   │   ├── worlds/
│   │   │   └── vlm_room.world                  ← 6x6 m room with 4 coloured objects
│   │   ├── tools/
│   │   │   ├── gemini_probe.py                 ← standalone CLI: test Gemini API on an image
│   │   │   └── save_frame.py                   ← ROS 2 subscriber: save one camera frame to PNG
│   │   ├── package.xml
│   │   └── setup.py
│   └── vlm_nav_robot_interfaces/               ← CMake package (service definition only)
│       ├── srv/FindAndGo.srv
│       ├── CMakeLists.txt
│       └── package.xml
├── maps/
│   └── vlm_room_20260420_210418.yaml           ← SLAM map saved from vlm_room.world
│       vlm_room_20260420_210418.pgm
└── docs/
    └── vlm_nav_robot.md                        ← this file
```

---

## 3. Simulation Environment

### World: `vlm_room.world`

A 6 × 6 m enclosed room with four distinctly coloured objects the robot can be sent to navigate to:

| Object | Type | World position (x, y) | Colour |
|---|---|---|---|
| `red_block` | 0.4 m cube | (1.5, 1.0) | Red |
| `green_block` | 0.4 m cube | (1.5, −1.2) | Green |
| `blue_cylinder` | r=0.2 h=0.6 cylinder | (−1.5, 1.3) | Blue |
| `yellow_cylinder` | r=0.2 h=0.6 cylinder | (−1.5, −1.0) | Yellow |

Robot spawn default: `(0, 0)` facing `+x` (east).

### Camera FoV and Object Visibility

The camera has a **90° horizontal FoV (±45°)** after the Week 10 fix (was 60°). From spawn at (0,0) facing +x:

| Object | Angle from forward | Visible from spawn? |
|---|---|---|
| red_block | +34° | Yes |
| green_block | −39° | Yes |
| blue_cylinder | +139° | No — behind robot |
| yellow_cylinder | −147° | No — behind robot |

To test blue/yellow, navigate the robot toward them first (or spawn at a rotated pose).

### Robot Spec

| Parameter | Value |
|---|---|
| Chassis | 0.30 × 0.25 × 0.08 m |
| Drive | Differential drive |
| LiDAR | 2D, 360°, range 0.1–10 m, 10 Hz |
| Camera | RGB-D, 640×480, 90° H-FoV, 0.1–10 m depth, 15 Hz |
| Camera mount height | ~0.15 m above base_link (front face) |

---

## 4. Service Interface

### `FindAndGo.srv`

```
# Request
string target          # natural language: "red box", "blue cylinder", etc.
---
# Response
bool   success
string reason          # human-readable: confidence, goal coords, nav status
geometry_msgs/PoseStamped goal   # the computed goal in map frame (zero if failed)
```

Service name: `/vlm_grounder/find_and_go`

**Success response example:**
```
success=True
reason="target='red block' conf=1.00 goal=(1.21,0.83) yaw=0.58 [sent to NavigateToPose]"
```

**Failure response examples:**
```
reason='no RGB/depth/camera_info received yet'          # grounder not ready
reason='low confidence 0.23 for red box — target may not be visible'  # below 0.5 threshold
reason='no valid depth in bbox (...) — target may be outside depth range or occluded'
reason='tf2 transform camera_optical_link → map failed: ...'  # AMCL not localised
```

---

## 5. Pipeline Detail

```
FindAndGo service call
        │
        ▼
Latest RGB + depth + camera_info (thread-safe snapshot)
        │
        ▼
PIL.Image → Gemini 2.5 Flash
  prompt: "Detect bbox of {target}"
  response schema: {label, box_2d:[y1,x1,y2,x2] 0–1000, confidence, notes}
  retries: 3× on HTTP 429/503 with exponential backoff
        │
        ├── confidence < 0.5 → FAIL
        ▼
Pixel bbox → depth sample
  Upper 60% of bbox, horizontally inset 20% (avoids floor leakage)
  Fallback: full bbox if upper region has no valid depth pixels
  Returns median of valid pixels (finite, > 0.05 m)
        │
        ├── no valid depth → FAIL
        ▼
Back-projection (camera optical frame)
  X = (u − cx) * d / fx
  Y = (v − cy) * d / fy
  Z = d
        │
        ▼
TF2 transform: camera_optical_link → map
  stamp=0 (use latest TF) — safe because robot is stationary during Gemini latency
        │
        ├── TF unavailable → FAIL
        ▼
Standoff calculation
  frac = (dist − standoff_m) / dist    [standoff_m = 0.3]
  goal_xy = robot_xy + frac * (target_xy − robot_xy)
  yaw = atan2(dy, dx)
        │
        ▼
NavigateToPose action goal → Nav2
  best-effort: if action server unavailable, still returns pose via service
        │
        ▼
FindAndGo.Response(success=True, reason=..., goal=PoseStamped)
```

---

## 6. Node Parameters

| Parameter | Default | Description |
|---|---|---|
| `confidence_threshold` | `0.5` | Minimum Gemini confidence to proceed |
| `standoff_m` | `0.3` | Distance to stop short of target (m) |
| `goal_frame` | `map` | TF frame for the output goal |
| `robot_base_frame` | `base_footprint` | Used to look up robot position for standoff calc |

Set via the launch file or `--ros-args -p key:=value`.

---

## 7. Environment Setup

### Required Python packages (once)

```bash
pip install --user google-genai pillow numpy
```

### GEMINI_API_KEY

The node will raise `RuntimeError` on startup if the key is not set. Add it to `~/.bashrc` for convenience:

```bash
echo 'export GEMINI_API_KEY="your-key-here"' >> ~/.bashrc
source ~/.bashrc
```

---

## 8. Build

```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash

# Always build interfaces package first
colcon build --packages-select vlm_nav_robot_interfaces
colcon build --packages-select vlm_nav_robot --symlink-install

source install/setup.bash
```

`--symlink-install` means Python file edits take effect immediately without rebuilding.

---

## 9. How to Launch and Test

### Full stack (4 terminals)

**Terminal 1 — Launch everything**
```bash
cd ~/slam_nav_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export GEMINI_API_KEY="your-key"
ros2 launch vlm_nav_robot vlm_full.launch.py
```

Wait for this sequence in the logs:
1. `[gzserver]` — Gazebo up (~2 s)
2. `[spawn_entity.py]` — robot spawned (~3 s)
3. `Managed nodes are active` from lifecycle_manager (~8–10 s)
4. `vlm_grounder ready` (~12 s)

**Terminal 2 — Set initial pose** (once per session, after lifecycle_manager says active)

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position:    {x: 0.0, y: 0.0,  z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0,    0,0,0,0,0,0,    0,0,0,0,0,0.06]
  }
}' --ros-args -p use_sim_time:=true
```

Wait for `Received initial pose` in Terminal 1 before calling the service.

**Terminal 3 — RViz2 (optional)**
```bash
source /opt/ros/humble/setup.bash && source ~/slam_nav_ws/install/setup.bash
ros2 run rviz2 rviz2
```
Set Fixed Frame to `map`. Add displays: Map, RobotModel, Path, LaserScan.

**Terminal 4 — Call the service**

```bash
source /opt/ros/humble/setup.bash && source ~/slam_nav_ws/install/setup.bash

# Targets visible from spawn (robot at origin facing +x):
ros2 service call /vlm_grounder/find_and_go vlm_nav_robot_interfaces/srv/FindAndGo "{target: 'red box'}"
ros2 service call /vlm_grounder/find_and_go vlm_nav_robot_interfaces/srv/FindAndGo "{target: 'green box'}"

# After driving robot back to origin or rotating to face -x:
ros2 service call /vlm_grounder/find_and_go vlm_nav_robot_interfaces/srv/FindAndGo "{target: 'blue cylinder'}"
ros2 service call /vlm_grounder/find_and_go vlm_nav_robot_interfaces/srv/FindAndGo "{target: 'yellow cylinder'}"
```

### Launch with a custom map

```bash
ros2 launch vlm_nav_robot vlm_full.launch.py map_yaml:=/path/to/your_map.yaml
```

### Launch headless (no Gazebo GUI)

```bash
ros2 launch vlm_nav_robot vlm_full.launch.py headless:=true
```

---

## 10. Pre-flight Tools

These standalone scripts are in `src/vlm_nav_robot/tools/` and do **not** require the full stack to be running.

### Test Gemini API without ROS

```bash
export GEMINI_API_KEY="your-key"
python3 src/vlm_nav_robot/tools/gemini_probe.py /path/to/image.png "red box"

# With bbox visualisation overlay:
python3 src/vlm_nav_robot/tools/gemini_probe.py /tmp/frame.png "red box" --draw /tmp/annotated.png
```

Output: latency, confidence, raw bbox coords, pixel bbox.

### Save a live camera frame from the sim

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
# With sim running in Terminal 1:
python3 src/vlm_nav_robot/tools/save_frame.py /tmp/frame.png
# Then probe it:
python3 src/vlm_nav_robot/tools/gemini_probe.py /tmp/frame.png "red box"
```

---

## 11. Sanity Check Commands

```bash
# Camera topics publishing?
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw

# Nav2 lifecycle active?
ros2 lifecycle get /bt_navigator

# Grounder service registered?
ros2 service list | grep find_and_go

# TF chain intact?
ros2 run tf2_tools view_frames   # produces /tmp/frames.pdf
ros2 run tf2_ros tf2_echo map base_footprint
```

---

## 12. Nav2 Configuration Summary

| Parameter | Value | Why |
|---|---|---|
| Planner | NavFn (Dijkstra) | Robust for simple rooms |
| Controller | DWB | Good obstacle avoidance |
| xy_goal_tolerance | 0.25 m | Generous for sim |
| yaw_goal_tolerance | 0.25 rad | ~14° — sufficient |
| max_vel_x | 0.26 m/s | Matches diff-drive limits |
| robot_radius | 0.22 m | Includes inflation |
| inflation_radius | 0.55 m | Keeps robot from walls |
| AMCL initial pose | (0, 0, 0) yaw=0 | Set in nav2_params.yaml as default; overridden by topic pub |
| use_sim_time | true | Always in Gazebo |

---

## 13. Progress Tracker

| Week | What Was Built | Status |
|---|---|---|
| **Week 9** | RGB-D camera URDF plugin, `vlm_demo.launch.py` sim, `gemini_probe.py` CLI preflight, `save_frame.py` capture tool. Verified `/camera/image_raw` and `/camera/depth/image_raw` publishing. Verified Gemini API key works and returns schema-validated JSON. | ✅ Done |
| **Week 10 Part A** | `vlm_grounder.py` node: Gemini detection, depth back-projection, TF2 map-frame transform, standoff goal, `NavigateToPose` action client, `FindAndGo` service. `vlm_nav_robot_interfaces` CMake package with `FindAndGo.srv`. | ✅ Done |
| **Week 10 Part B** | `vlm_full.launch.py` full stack: Gazebo + robot_state_publisher + Nav2 (all 7 lifecycle nodes) + vlm_grounder under lifecycle delay. End-to-end verified: service call → Gemini → depth → TF → NavigateToPose → robot drives. | ✅ Done |

---

## 14. What Is Left To Do

| Task | Priority | Notes |
|---|---|---|
| Validate all four targets | High | red ✅, green ✅ (after FoV fix), blue/yellow need robot facing backward |
| Record demo video | High | 60–90 s showing: launch → initial pose → service call → robot drives to target |
| Commit Week 10 code | High | Both parts A and B in one commit |
| Optional: Docker image | Low | Mirror the Project 2 Docker pattern for reproducibility |
| Optional: rotate-to-search | Low | Add a spin behavior when confidence < threshold before declaring failure |

---

## 15. Known Issues and Fixes Applied

### Fix 1 — Depth failure when bbox clips top of image

**Symptom:** `no valid depth in bbox (64,0,154,48)` where `y=0` on the first service call.

**Root cause:** Gemini sometimes places the bbox right at the top image edge (`y1=0`). The depth sampler takes the **upper 60%** of the bbox to avoid floor pixels leaking into the sample. When `y1=0`, this upper region spans the very top rows of the depth frame where Gazebo often returns zeros.

**Fix applied** (`vlm_grounder.py:_sample_depth`): if the upper-60% region has no valid depth pixels, fall back to sampling the full bbox before returning `None`. First-call failures are now recovered automatically.

### Fix 2 — Camera FoV too narrow for side objects

**Symptom:** `green box` returned `confidence=0.00` even with the green block in the world.

**Root cause:** Camera H-FoV was 60° (±30°). The green block is at −39° from forward at spawn, the red block at +34°. Both were at or just outside the ±30° boundary.

**Fix applied** (`robot.urdf.xacro`): FoV changed from `1.0472` rad (60°) to `1.5708` rad (90°, ±45°). Both red and green blocks are now comfortably inside view from the default spawn pose.

### Known minor issue — First call after startup sometimes fails

**Symptom:** Very first `FindAndGo` call within ~2 s of `vlm_grounder ready` returns `no RGB/depth/camera_info received yet`.

**Root cause:** The depth/RGB subscribers haven't received their first message yet — Gazebo camera plugins have a brief warm-up period.

**Workaround:** Call the service a second time. No code fix needed — this is expected transient behaviour.

---

## 16. Bugs to NOT Reintroduce

| Bug | What breaks | Fix |
|---|---|---|
| `robot_description` param without `ParameterValue(value_type=str)` | Launch file type error | Always wrap in `ParameterValue` |
| `nav2_recoveries` package name | Package not found on Humble | Use `nav2_behaviors` |
| `recovery_plugins` key in params | Lifecycle activation fail | Use `behavior_plugins` |
| `default_nav_to_pose_bt_xml: ""` | bt_navigator rejects every goal | Must point at a real XML path |
| `nav2_drive_on_heading_action_bt_node` in plugin list | Activation fail — lib not found | Use `nav2_drive_on_heading_bt_node` (no `_action` infix) |
| Depth sampling: no fallback when bbox clips image edge | First call fails on top-edge bbox | Full-bbox fallback now in place |
| Camera FoV 60° for objects at ±35–40° | Objects at confidence=0.00 | FoV is now 90° |

---

## 17. File-by-File Reference

| File | Purpose |
|---|---|
| `vlm_nav_robot/vlm_grounder.py` | Core node. FindAndGo service handler, Gemini client, depth sampler, TF2 transform, Nav2 action client. Entry point: `vlm_grounder`. |
| `launch/vlm_full.launch.py` | Full stack launch: Gazebo + RSP + spawn + Nav2 (7-node lifecycle) + vlm_grounder. Nav2 delayed 6 s, grounder delayed 12 s. |
| `launch/vlm_demo.launch.py` | Sim-only launch (Week 9): Gazebo + RSP + spawn. No Nav2, no grounder. Used for camera verification. |
| `config/nav2_params.yaml` | All Nav2 node parameters. AMCL default initial pose (0,0,0). DWB controller tuned for 0.26 m/s max. |
| `description/robot.urdf.xacro` | Robot URDF. Diff-drive, 2D LiDAR, RGB-D camera at 90° FoV. |
| `worlds/vlm_room.world` | 6×6 m Gazebo world with 4 coloured objects. |
| `tools/gemini_probe.py` | Standalone CLI — test Gemini API key and bbox output on any PNG. No ROS needed. |
| `tools/save_frame.py` | ROS 2 one-shot subscriber — saves one `/camera/image_raw` frame to PNG. |
| `vlm_nav_robot_interfaces/srv/FindAndGo.srv` | Service definition: `string target` → `bool success, string reason, PoseStamped goal`. |
