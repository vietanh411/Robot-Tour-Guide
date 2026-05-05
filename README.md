# Robot Tour Guide

A hybrid-architecture tour-guide stack for the **OU TurtleBot 4**, written for
ROS 2 (Jazzy) + Gazebo Harmonic. The deliberative layer plans an optimal
visiting order over a set of points-of-interest, and the executive layer
classifies Nav2 failures (person blocking, furniture moved, door closed,
unreachable POI) and chooses an appropriate recovery — instead of blindly
retrying.

A printable **ArUco** marker is taped near each tour stop. When the robot's
OAK-D camera sees one, the matching description (from `landmarks.yaml`) is
printed in the terminal and spoken via TTS.

---

## Repository layout

```
Robot-Tour-Guide/
└── ros2_ws/
    └── src/
        └── robot_tour_guide/
            ├── package.xml
            ├── setup.py
            ├── robot_tour_guide/         # node implementations
            │   ├── aruco_detector.py
            │   ├── semantic_perception.py
            │   ├── world_model.py
            │   ├── tour_planner.py
            │   ├── executive.py
            │   ├── narrator.py
            │   ├── landmark_announcer.py
            │   └── safety_monitor.py
            ├── launch/
            │   ├── tour_guide.launch.py        # full stack
            │   ├── tour_guide_sim.launch.py    # full stack, sim wrapper
            │   └── perception_only.launch.py   # ArUco-only demo path
            └── config/
                ├── landmarks.yaml              # marker id -> description
                ├── pois.yaml                   # tour stops with poses
                └── params.yaml                 # all node parameters
```

## Architecture (one-screen view)

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

Topic / data summary:

| Topic                     | Type            | Producer                | Consumer            |
|---------------------------|-----------------|-------------------------|---------------------|
| `/landmarks/detected`     | `std_msgs/String` (JSON) | `aruco_detector`     | `world_model`, `executive`, `landmark_announcer` |
| `/landmarks/annotated`    | `sensor_msgs/Image`      | `aruco_detector`     | RViz preview        |
| `/world/objects`          | `std_msgs/String` (JSON) | `semantic_perception`| `world_model`       |
| `/world/state`            | `std_msgs/String` (JSON) | `world_model`        | `executive`         |
| `/world/markers`          | `MarkerArray`            | `world_model`        | RViz                |
| `/tour/current_plan`      | `std_msgs/String` (JSON) | `tour_planner`       | `executive`         |
| `/tour/replan_request`    | `std_msgs/String` (JSON) | `executive`          | `tour_planner`      |
| `/tour/drop_poi`          | `std_msgs/String` (int)  | `executive`          | `tour_planner`      |
| `/tour/narration`         | `std_msgs/String`        | `executive`, `landmark_announcer` | `narrator` |
| `/tour/status`            | `std_msgs/String` (JSON) | `executive`          | external monitors   |
| `navigate_to_pose`        | Nav2 action              | `executive`          | Nav2                |

> JSON-in-`std_msgs/String` is used to avoid the build-system complexity of a
> separate `robot_tour_guide_msgs` package. Schemas are documented inline in
> each node.

## Failure classifier (the deliberative core)

When Nav2 reports failure, `executive.classify_failure()` inspects the latest
world snapshot and selects a recovery:

| Detected condition                                         | Class                | Recovery                                  |
|------------------------------------------------------------|----------------------|-------------------------------------------|
| Person within `person_blocking_distance_m`                 | `PERSON_BLOCKING`    | Say "excuse me", wait `person_block_wait_s`, retry |
| New furniture-class cluster appeared in last 8 s           | `FURNITURE_MOVED`    | Request replan from current pose          |
| No `door_open` cluster in forward arc, retries exhausted   | `DOOR_CLOSED`        | Drop POI, replan, narrate the skip        |
| Retries exhausted with no other signal                     | `POI_UNREACHABLE`    | Drop POI, narrate, continue tour          |
| Anything else                                              | `UNKNOWN`            | Short pause and retry once                |

---

## Build

The robot ships with ROS 2 Jazzy, OpenCV ≥ 4.7 (which has the new
`cv2.aruco.ArucoDetector`), and Nav2 already installed. On the desktop:

```bash
# 1. Get the code
cd ~
git clone https://github.com/<your-org>/Robot-Tour-Guide.git
cd Robot-Tour-Guide/ros2_ws

# 2. (Optional) install YOLO + TTS for the perception layer + narrator
pip install --user ultralytics pyttsx3

# 3. Build
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The first time `semantic_perception` runs it will auto-download
`yolov8n.pt` (~6 MB) into the working directory. If you have no GPU, `yolov8n`
runs at ~5 Hz on a modern laptop CPU.

---

## Print the markers (one-time)

Generate a 4×4 ArUco marker for each landmark id in `landmarks.yaml`:

1. Visit <https://chev.me/arucogen/>.
2. Set **Dictionary = 4x4 (50, 100, 250, 1000)**.
3. Enter the marker ID and a side length matching `aruco_detector.marker_size_m`
   (default **0.10 m**, i.e. 10 cm). Use a ruler to confirm the printed size.
4. Print, cut, and tape one marker at chest height near each tour stop.

> A wrong `marker_size_m` will not break detection but will give you wrong
> distances, which the arrival logic depends on.

---

## Demo procedure (REPF B4)

You have **two demo paths** — pick one based on time and risk tolerance.

### Path A — Quickest: ArUco-only demo (5 min setup)

Shows the symbol-to-description feature without Nav2 or planning. You drive,
the robot recognises markers and announces them.

**On the Pi (one SSH session per command):**

```bash
ssh student@<robotName>.cs.nor.ou.edu
ros2 launch turtlebot4_bringup robot.launch.py        # if not already up
ros2 launch turtlebot4_bringup oakd.launch.py         # ensure camera is on
ros2 service call /start_motor std_srvs/srv/Empty "{}"
```

**On the desktop:**

```bash
# 1. Connect to the robot
robot-setup.sh        # follow prompts; enter the robot name

# 2. Build + source (one-time per shell)
cd ~/Robot-Tour-Guide/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Launch perception-only stack
ros2 launch robot_tour_guide perception_only.launch.py

# 4. In a NEW terminal: drive the robot manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# 5. (Optional) RViz so the audience sees what the robot sees
ros2 launch turtlebot4_viz view_robot.launch.py
# In RViz add an Image display and set Topic = /landmarks/annotated
```

Drive close to a marker (≤ 1.5 m), face it for ~1 s, and listen for the
description. The cooldown is 12 s by default, so you won't get spammed if you
linger. Switch to a different marker and the new description plays.

### Path B — Full tour: planner + executive + Nav2 (15 min setup)

Shows the complete deliberative + reactive system. Requires a map of REPF B4.

**One-time prerequisite — make a map:**

```bash
# Pi
ros2 service call /start_motor std_srvs/srv/Empty "{}"

# Desktop
ros2 launch turtlebot4_navigation slam.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
# Drive around REPF B4 until the map looks complete in RViz.

ros2 run nav2_map_server map_saver_cli -f ~/repf_b4_map
```

Then update `pois.yaml` with the actual `(x, y, yaw)` of each landmark in your
saved map (use RViz "Publish Point" or read coordinates off the map).

**Live demo, in five terminals:**

```bash
# Terminal 1 (Pi - SSH)
ros2 launch turtlebot4_bringup robot.launch.py

# Terminal 2 (Pi - SSH)
ros2 launch turtlebot4_bringup oakd.launch.py
ros2 service call /start_motor std_srvs/srv/Empty "{}"

# Terminal 3 (Desktop)
robot-setup.sh
ros2 launch turtlebot4_navigation localization.launch.py \
     map:=$HOME/repf_b4_map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py

# Terminal 4 (Desktop)
robot-setup.sh
ros2 launch turtlebot4_viz view_robot.launch.py
# In RViz: "2D Pose Estimate" to set initial pose.
# Add MarkerArray on /world/markers and Image on /landmarks/annotated.

# Terminal 5 (Desktop) - the tour itself
robot-setup.sh
cd ~/Robot-Tour-Guide/ros2_ws && source install/setup.bash
ros2 launch robot_tour_guide tour_guide.launch.py
```

The robot will auto-plan a tour from its current pose, drive to each POI,
narrate when it sees the marker, and skip stops it cannot reach.

### Optional: enable the safety monitor on the Pi

Run on the Pi (not the desktop) to keep the e-stop reflex tight:

```bash
ros2 launch robot_tour_guide tour_guide.launch.py enable_safety_monitor:=true
```

(Run only the safety_monitor node on the Pi; everything else stays on the
desktop. The launch file will start *all* nodes — for a Pi-only safety
monitor, run `ros2 run robot_tour_guide safety_monitor --ros-args
--params-file <path>` directly.)

---

## Recommended demo script (~3 minutes)

Memorise this sequence so you can talk while the robot drives:

| t (s) | Action                                        | What you say                                                |
|-------|-----------------------------------------------|-------------------------------------------------------------|
| 0     | Show RViz with the planned tour overlaid      | "The planner solved a TSP over five POIs."                  |
| 15    | Robot reaches first marker; description plays | "ArUco marker confirms arrival, narration is data-driven." |
| 45    | Step in front of the robot                    | "Person detected — watch the executive say excuse me."     |
| 60    | Step out, robot resumes                       | "It went back to the same POI rather than skipping."       |
| 90    | Move a chair onto the next path while moving  | "FurnitureMoved class triggers a replan."                  |
| 120   | Block the last POI completely with a board    | "Two retries fail; POI is dropped and tour finishes."      |
| 180   | Robot says "Tour complete."                   | Wrap.                                                       |

---

## Troubleshooting

| Symptom                                            | Fix                                                                    |
|----------------------------------------------------|------------------------------------------------------------------------|
| `aruco_detector` logs "Camera intrinsics received" never appears | Confirm `oakd.launch.py` is running on the Pi.                  |
| Markers detected but distance is wildly wrong      | `marker_size_m` ≠ printed marker side length.                          |
| `executive` never leaves `IDLE`                    | `tour_planner` published an empty plan — check `pois.yaml` is loaded.  |
| Nav2 keeps failing on the same POI                 | Set the POI's `(x, y)` more carefully or drop it from `pois.yaml`.     |
| YOLO crashes on import                             | `pip install ultralytics` on the desktop, not the Pi.                  |
| TTS silent                                         | `pip install pyttsx3`; on Linux also `sudo apt install espeak`.        |
| `robot-setup.sh` doesn't expose topics             | `ros2 daemon stop && ros2 daemon start`, then re-run.                  |

---

## Code adoption note (for the bonus)

Three components in this stack are designed to be reused as-is by other
teams without depending on the rest:

* **`aruco_detector`** — drop-in OAK-D ArUco recogniser; publishes JSON.
* **`landmark_announcer`** — symbol→description with debounce, no planner needed.
* **`tour_planner`** — pure-Python TSP+2-opt over a YAML POI file.

To consume them, pull just those files and the `landmarks.yaml` /
`pois.yaml` schemas — no custom messages required.

---

## License

MIT (see `LICENSE`).
