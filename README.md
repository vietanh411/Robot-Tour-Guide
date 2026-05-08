# Robot Tour Guide

A hybrid-architecture tour-guide stack for the **OU TurtleBot 4** (ROS 2
Jazzy). The robot drives a planned route through four landmarks in a mapped
indoor space, prints/speaks each landmark's description, then returns to the
starting point. Nav2 handles obstacle and human avoidance during motion;
when navigation fails the executive classifies the cause (person blocking,
furniture moved, door closed, POI unreachable) and replans accordingly.

A printed **ArUco** marker is taped near each tour stop. When the OAK-D
camera sees one, the matching description from `landmarks.yaml` is published
on `/tour/narration` and either printed in green text or spoken via TTS.

---

## Repository layout

```
Robot-Tour-Guide/
├── README.md
├── WRITTEN_REPORT.md
├── Robot_Tour_Guide_Poster.pptx
├── LICENSE
└── ros2_ws/
    └── src/
        └── robot_tour_guide/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/robot_tour_guide
            ├── robot_tour_guide/
            │   ├── aruco_detector.py
            │   ├── semantic_perception.py
            │   ├── world_model.py
            │   ├── tour_planner.py
            │   ├── executive.py
            │   ├── narrator.py
            │   └── safety_monitor.py
            ├── launch/
            │   └── tour_guide.launch.py
            └── config/
                ├── landmarks.yaml
                ├── pois.yaml
                └── params.yaml
```

## Architecture

```
Deliberative      tour_planner   ──────────► /tour/current_plan
                       ▲                              │
                       │  /tour/replan_request        ▼
Executive         executive  ◄── /world/state ──  world_model
                       │  ▲                            ▲
       narration ──────┘  └─ /landmarks/detected ──┐   │
                       ▼                           │   │  /world/objects
Reactive       Nav2 (NavigateToPose) + safety_monitor  │
                       ▲                           │   │
Perception     aruco_detector ◄── camera           semantic_perception
                                                       ▲
                                                   /scan + camera
```

| Topic                     | Type                       | Producer            | Consumer              |
|---------------------------|----------------------------|---------------------|-----------------------|
| `/landmarks/detected`     | `std_msgs/String` (JSON)   | `aruco_detector`    | `world_model`, `executive` |
| `/landmarks/annotated`    | `sensor_msgs/Image`        | `aruco_detector`    | RViz preview          |
| `/world/objects`          | `std_msgs/String` (JSON)   | `semantic_perception` | `world_model`       |
| `/world/state`            | `std_msgs/String` (JSON)   | `world_model`       | `executive`           |
| `/world/markers`          | `visualization_msgs/MarkerArray` | `world_model` | RViz                  |
| `/tour/current_plan`      | `std_msgs/String` (JSON)   | `tour_planner`      | `executive`           |
| `/tour/replan_request`    | `std_msgs/String` (JSON)   | `executive`         | `tour_planner`        |
| `/tour/drop_poi`          | `std_msgs/String` (int)    | `executive`         | `tour_planner`        |
| `/tour/narration`         | `std_msgs/String`          | `executive`         | `narrator`            |
| `/tour/status`            | `std_msgs/String` (JSON)   | `executive`         | external monitors     |
| `navigate_to_pose`        | Nav2 action                | `executive`         | Nav2                  |

> JSON-in-`std_msgs/String` is used to avoid the build-system overhead of a
> separate `robot_tour_guide_msgs` package.

## Executive states

| State            | Meaning                                                       |
|------------------|---------------------------------------------------------------|
| `IDLE`           | Waiting for a plan.                                           |
| `NAVIGATING`     | Nav2 goal active for the current POI.                         |
| `AT_POI`         | Reached by Nav2 success or matching ArUco landmark in range.  |
| `NARRATING`      | Speaking/printing the landmark description.                   |
| `RECOVERING`     | Handling a classified Nav2 failure.                           |
| `RETURNING_HOME` | Driving back to the recorded start pose.                      |
| `DONE`           | Tour finished.                                                |

The first AMCL pose received is captured as the **home pose**. After the
last POI's description finishes (or after the plan is exhausted by drops),
the executive announces *"Tour complete. Returning to the starting point."*,
sends a Nav2 goal back to that pose, and concludes with *"I have returned
to the starting point. Thank you for visiting!"*

## Failure classifier

When Nav2 reports failure, `executive.classify_failure()` inspects the
latest world snapshot and picks a recovery:

| Detected condition                                         | Class                | Recovery                                             |
|------------------------------------------------------------|----------------------|------------------------------------------------------|
| Person within `person_blocking_distance_m`                 | `PERSON_BLOCKING`    | Say "excuse me", wait `person_block_wait_s`, retry  |
| New furniture-class cluster appeared in last 8 s           | `FURNITURE_MOVED`    | Request replan from current pose                     |
| No `door_open` cluster forward, retries exhausted          | `DOOR_CLOSED`        | Drop POI, replan, narrate the skip                  |
| Retries exhausted with no other signal                     | `POI_UNREACHABLE`    | Drop POI, narrate, continue tour                     |
| Anything else                                              | `UNKNOWN`            | Short pause and retry                                |

---

## Build

The OU TurtleBot 4 desktops ship with ROS 2 Jazzy, OpenCV ≥ 4.7 (with the
new `cv2.aruco` API), and Nav2 / `turtlebot4_navigation` already installed.

```bash
cd ~/Robot-Tour-Guide/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Optional Python deps (only needed for richer perception and TTS):

```bash
pip install --user ultralytics pyttsx3
```

If `ultralytics` is missing, `semantic_perception` runs in LiDAR-only mode
and the rest of the stack still works.

## Print the markers (one-time)

Generate one ArUco marker per landmark id in
[`config/landmarks.yaml`](ros2_ws/src/robot_tour_guide/config/landmarks.yaml):

1. Visit <https://chev.me/arucogen/>.
2. **Dictionary = 4x4 (50, 100, 250, 1000)**.
3. Side length = `aruco_detector.marker_size_m` from
   [`config/params.yaml`](ros2_ws/src/robot_tour_guide/config/params.yaml)
   (default **0.10 m**). Confirm with a ruler — a wrong size will not break
   detection but will give wrong distance measurements, and the
   landmark-arrival logic relies on distance.
4. Print, cut, tape one near each tour stop at roughly camera height
   (~25–35 cm) on a flat vertical surface.

---

## Demo procedure

### Step 1 — Map the room (one-time per demo space)

On the Pi:

```bash
ssh student@<robot>.cs.nor.ou.edu
ros2 launch turtlebot4_bringup robot.launch.py
ros2 service call /start_motor std_srvs/srv/Empty "{}"
```

On the desktop:

```bash
robot-setup.sh
ros2 launch turtlebot4_navigation slam.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Drive around until the map looks complete in RViz, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
# produces ~/my_map.pgm and ~/my_map.yaml
```

Update [`config/pois.yaml`](ros2_ws/src/robot_tour_guide/config/pois.yaml)
with the `(x, y, yaw)` of each landmark in **your** map (read off RViz with
"Publish Point", or eyeball from the saved map). Rebuild after editing
configs:

```bash
cd ~/Robot-Tour-Guide/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Step 2 — Run the demo (Nav2 + localization separate)

The TurtleBot 4 navigation stack is best brought up in its own terminals
rather than included from our launch file — it keeps the lifecycle managers
independent and makes it obvious which process logs belong to which layer.
Every desktop terminal must run `robot-setup.sh` first.

**Pi (3 SSH sessions, in order):**

```bash
ssh student@<robot>.cs.nor.ou.edu              # password: student

# Pi-T1
ros2 launch turtlebot4_bringup robot.launch.py

# Pi-T2
ros2 launch turtlebot4_bringup oakd.launch.py

# Pi-T3
ros2 service call /start_motor std_srvs/srv/Empty "{}"
```

**Desktop (4 terminals):**

```bash
# Desk-T1 — Localization (map_server + AMCL)
robot-setup.sh
ros2 launch turtlebot4_navigation localization.launch.py \
     map:=$HOME/Robot-Tour-Guide/ros2_ws/src/robot_tour_guide/maps/repf_b4_map.yaml
# WAIT for: "Managed nodes are active"
```

```bash
# Desk-T2 — Nav2 (planner + controller + recoveries)
robot-setup.sh
ros2 launch turtlebot4_navigation nav2.launch.py
# WAIT for: "Managed nodes are active"
```

```bash
# Desk-T3 — RViz
robot-setup.sh
ros2 launch turtlebot4_viz view_robot.launch.py
```

In RViz **before launching the tour**:

1. **Global Options → Fixed Frame = `map`**.
2. The **Map** display should populate. If it's blank, set
   **Topic = `/map`** and **Durability Policy = Transient Local**.
3. Click **2D Pose Estimate** on the toolbar, then click+drag on the
   robot's true location pointing the direction it's actually facing.
4. The laser scan should snap onto the walls. If it doesn't, click
   **2D Pose Estimate** again and try a slightly different position/heading.

Sanity-check that the navigation action is alive:

```bash
ros2 action list | grep navigate
# should print: /navigate_to_pose
```

```bash
# Desk-T4 — The tour itself (skip Nav2 since we already brought it up)
robot-setup.sh
cd ~/Robot-Tour-Guide/ros2_ws && source install/setup.bash
ros2 launch robot_tour_guide tour_guide.launch.py bringup_nav2:=false
```

The tour begins:

1. `tour_planner` orders the four POIs (nearest-neighbor + 2-opt).
2. `executive` sends the first Nav2 goal and captures the AMCL pose as **home**.
3. At each stop the matching ArUco marker is detected and the description
   in `landmarks.yaml` is announced.
4. If a person stands in the path, Nav2 will try to avoid them; if it
   fails, the executive says *"Excuse me, may I please come through?"*,
   waits, then retries.
5. If a chair is moved into the path, the executive requests a replan.
6. If a stop is unreachable after retries, it is skipped.
7. After the last stop, the robot announces *"Tour complete. Returning to
   the starting point."*, drives back to where you placed the initial
   pose, and concludes with *"I have returned to the starting point.
   Thank you for visiting!"*

### When done

```bash
# Pi-T3
ros2 service call /stop_motor std_srvs/srv/Empty "{}"
```

Dock the robot. **Always stop the LiDAR motor before docking.**

### Tuning robot speed during the demo

The default Nav2 cruise speed (~0.5 m/s) is too quick for a tight indoor
demo. Drop it at runtime — takes effect on the next goal, no rebuild:

```bash
ros2 param set /controller_server FollowPath.desired_linear_vel 0.15
ros2 param set /velocity_smoother max_velocity "[0.15, 0.0, 1.0]"
ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel 0.4
```

These reset whenever Nav2 restarts; re-apply after any restart of Desk-T2.

### Launch arguments (`tour_guide.launch.py`)

| Argument                | Default                       | Meaning                                                                  |
|-------------------------|-------------------------------|--------------------------------------------------------------------------|
| `map`                   | packaged `maps/repf_b4_map.yaml` | Absolute path to the map YAML used by AMCL when `bringup_nav2:=true`. |
| `params_file`           | packaged `params.yaml`        | Override the parameter file used by every tour-guide node.               |
| `bringup_nav2`          | `true`                        | Set to `false` when Nav2 + AMCL are launched in their own terminals.     |
| `enable_safety_monitor` | `true`                        | Set to `false` to disable the LiDAR forward-arc emergency stop.          |

---

## Configuration files

| File | Purpose |
|---|---|
| [`config/landmarks.yaml`](ros2_ws/src/robot_tour_guide/config/landmarks.yaml) | ArUco id → landmark name and narration text. |
| [`config/pois.yaml`](ros2_ws/src/robot_tour_guide/config/pois.yaml) | Tour stops with `(x, y, yaw)` in the map frame, plus matching landmark id. |
| [`config/params.yaml`](ros2_ws/src/robot_tour_guide/config/params.yaml) | Centralized ROS parameters for every node. |

Edit these YAML files instead of code whenever possible. After changing
them, rebuild the workspace (`colcon build --symlink-install`) so the
installed `share/` copies update.

## Troubleshooting

| Symptom                                           | Fix                                                                    |
|---------------------------------------------------|------------------------------------------------------------------------|
| `aruco_detector` never logs "Camera intrinsics received" | Confirm `oakd.launch.py` is running on the Pi.                  |
| Markers detected but distance wildly wrong        | `marker_size_m` ≠ printed marker side length.                          |
| `executive` never leaves `IDLE`                   | Tour planner published an empty plan — check `pois.yaml` is loaded.    |
| Robot never returns home                          | AMCL pose was never received before the last POI; set initial pose in RViz earlier next time. |
| Nav2 keeps failing on the same POI                | Re-check the POI's `(x, y)` against your map, or drop it from `pois.yaml`. |
| YOLO crashes on import                            | `pip install ultralytics` on the desktop, not the Pi.                  |
| TTS silent                                        | `pip install pyttsx3`; on Linux also `sudo apt install espeak`.        |
| `robot-setup.sh` doesn't expose topics            | `ros2 daemon stop && ros2 daemon start`, then re-run.                  |

---

## License

MIT (see `LICENSE`).
