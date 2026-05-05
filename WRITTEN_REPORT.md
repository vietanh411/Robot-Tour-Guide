# A Hybrid ROS 2 Architecture for a TurtleBot 4 Robot Tour Guide

**Authors:** Robot Tour Guide Team  
**Course:** Intelligent Mobile Robotics  
**Date:** May 5, 2026

## Abstract

This project addresses the problem of enabling an OU TurtleBot 4 to act as a semi-autonomous indoor tour guide in a classroom-scale environment. The problem is worth studying because a tour-guide robot must do more than navigate between waypoints: it must recognize meaningful locations, interact intelligibly with people, and recover from common navigation failures in a way that appears deliberate rather than repetitive. We implemented a hybrid ROS 2 software stack that combines ArUco landmark recognition, optional semantic perception, a time-decayed world model, a TSP-style tour planner, a Nav2-based executive, narration, and a reactive safety monitor. The resulting system provides two demonstration paths: a low-risk ArUco-only narration demo and a full tour demo in which the robot plans an ordered route through points of interest, drives with Nav2, confirms arrivals through landmarks, narrates stops, and classifies selected failures as person blocking, furniture moved, door closed, or unreachable point of interest. We conclude that a hybrid architecture is an appropriate design for the tour-guide mission because it separates long-horizon route choice, local navigation, symbolic landmark recognition, human-facing narration, and short-latency safety responses while keeping the system understandable and reusable.

## 1. Introduction

Mobile robots that operate around people need software that balances goal-directed behavior with immediate response to changing conditions. A robot tour guide is a useful example of this balance. It must choose where to go, travel through an environment, recognize when it has reached a meaningful place, explain that place to visitors, and recover when the world does not match the plan. In a classroom or hallway-scale setting, failures are likely: people may stand in the robot's path, furniture may be moved, a route may become blocked, or a navigation goal may be unreachable because the waypoint was entered incorrectly. A simple waypoint follower can attempt the tour, but when something goes wrong it often has little mission-level context for deciding whether to wait, replan, skip a stop, or ask for help.

The purpose of this project was to design and implement software that helps an OU TurtleBot 4 act as a robot tour guide. The system targets ROS 2 Jazzy and Gazebo Harmonic, runs on top of TurtleBot 4 bringup and Nav2, and uses the robot's existing OAK-D camera and LiDAR without physical modification. The main contribution is not a new low-level controller, but a hybrid tour-guide architecture that connects symbolic landmark recognition, deliberative tour planning, semantic failure classification, and human-facing narration.

The project follows a hybrid robotics paradigm. The deliberative layer plans the visiting order over points of interest. The executive layer dispatches goals to Nav2, watches for landmark confirmations, publishes narration, and interprets failures. The reactive layer includes Nav2's local collision avoidance and a scan-based safety monitor that publishes zero velocity when an obstacle is too close in front of the robot. This separation makes the system easier to reason about because each layer has a distinct time scale and responsibility.

The software also emphasizes practical demonstration. The repository includes a full-stack launch file for the complete tour and a perception-only launch file for a lower-risk demo in which the robot is manually driven past printed ArUco markers and announces the matching descriptions. This second path is important because it demonstrates the symbol-to-description part of the tour-guide mission even if full autonomous navigation is not available during a live demo.

## 2. Background and Related Work

The architecture builds on several established robotics ideas and software systems.

ROS 2 provides the communication and packaging framework. ROS 2 Jazzy is the target distribution for this project, and its supported Gazebo pairing is Gazebo Harmonic. This matters because the assignment requires compatibility with ROS 2 and Gazebo Harmonic, and because TurtleBot 4 support is commonly organized around ROS 2 launch files, topics, parameters, and actions.

Nav2 supplies the navigation substrate. In this project, the executive sends `NavigateToPose` goals to the Nav2 action server. Nav2 already performs mapping, localization, path planning, control, obstacle avoidance, and lower-level recovery behaviors. Our work adds a mission-level executive above Nav2. Instead of treating all Nav2 failures the same way, the executive inspects the latest world model and chooses a recovery that is meaningful for the tour mission.

ArUco markers provide robust symbolic landmark recognition. ArUco markers are square fiducial markers with encoded identifiers that can be detected from camera images. They are appropriate for this project because the robot tour guide does not need general visual place recognition to demonstrate symbol-to-description behavior. A printed marker near each stop gives the robot a reliable, inspectable way to identify the current landmark and trigger the correct narration.

The project also borrows from hybrid and layered robot-control ideas. Brooks argued for layered robot control in which useful behavior emerges from multiple interacting layers rather than from a single monolithic planner. This project uses a related but mission-specific separation: reactive safety and Nav2 control run continuously, an executive handles state transitions and recoveries, and the planner computes a tour order over known points of interest. The result is simpler than a fully general robot architecture but richer than a pure reactive system or a pure waypoint list.

## 3. System Overview

The repository is a ROS 2 Python package named `robot_tour_guide`. It contains eight runtime nodes:

| Node | Responsibility |
| --- | --- |
| `aruco_detector` | Detects printed ArUco markers in the OAK-D RGB image and publishes JSON detections. |
| `semantic_perception` | Uses optional YOLO and LiDAR clustering to publish people, furniture candidates, door-open hints, and obstacles. |
| `world_model` | Maintains a time-decayed snapshot of recent objects and landmarks. |
| `tour_planner` | Loads POIs from YAML, plans an ordered tour, and supports replanning and dropped POIs. |
| `executive` | Runs the tour state machine, sends Nav2 goals, confirms arrivals, narrates stops, and classifies failures. |
| `landmark_announcer` | Provides a lightweight ArUco-only narration path without the planner or executive. |
| `narrator` | Prints narration and optionally speaks it through text-to-speech. |
| `safety_monitor` | Publishes zero velocity when a LiDAR return is too close in the forward arc. |

The nodes communicate using standard ROS messages. To avoid the overhead of defining a custom message package, structured information is sent as JSON inside `std_msgs/String`. This choice is not ideal for every production system, but it is a reasonable tradeoff for a course project because the message schemas are simple, easy to inspect with `ros2 topic echo`, and do not require another build target.

The major topics are:

| Topic | Type | Producer | Consumer |
| --- | --- | --- | --- |
| `/landmarks/detected` | JSON in `std_msgs/String` | `aruco_detector` | `world_model`, `executive`, `landmark_announcer` |
| `/landmarks/annotated` | `sensor_msgs/Image` | `aruco_detector` | RViz or image viewers |
| `/world/objects` | JSON in `std_msgs/String` | `semantic_perception` | `world_model` |
| `/world/state` | JSON in `std_msgs/String` | `world_model` | `executive` |
| `/world/markers` | `visualization_msgs/MarkerArray` | `world_model` | RViz |
| `/tour/current_plan` | JSON in `std_msgs/String` | `tour_planner` | `executive` |
| `/tour/replan_request` | JSON in `std_msgs/String` | `executive` | `tour_planner` |
| `/tour/drop_poi` | POI id in `std_msgs/String` | `executive` | `tour_planner` |
| `/tour/narration` | `std_msgs/String` | `executive`, `landmark_announcer` | `narrator` |
| `/tour/status` | JSON in `std_msgs/String` | `executive` | External monitors |
| `navigate_to_pose` | Nav2 action | `executive` | Nav2 |

## 4. Approach and Methods

### 4.1 Hardware and Platform Assumptions

The system is designed for the OU TurtleBot 4 with its standard sensors. It assumes that TurtleBot 4 bringup publishes the OAK-D RGB image stream, camera info, LiDAR scan, transforms, and robot state. The full autonomous tour also assumes that Nav2 and either SLAM or localization with a saved map are already running. The software does not require physical modifications to the robot.

The primary deployment environment is ROS 2 Jazzy with Gazebo Harmonic for simulation. The package is implemented with `ament_python`, declares ROS dependencies in `package.xml`, and installs launch files and YAML configuration through `setup.py`.

### 4.2 Landmark Detection

The `aruco_detector` node subscribes to the OAK-D RGB image and camera-info topics. Once camera intrinsics are available, each image is converted with `cv_bridge`, converted to grayscale, and passed to OpenCV's ArUco detector. The detector supports multiple dictionaries, including `4x4_50`, `4x4_100`, `4x4_250`, and `4x4_1000`. The active configuration uses `4x4_1000`, so the printed markers should be generated from the same dictionary.

For each detected marker, the node estimates pose with `cv2.solvePnP` using the configured physical marker side length. It then computes:

| Field | Meaning |
| --- | --- |
| `id` | ArUco marker identifier. |
| `distance_m` | Euclidean distance from camera to marker. |
| `bearing_rad` | Horizontal bearing relative to the camera optical axis. |
| `pose_camera` | Marker translation in the camera frame. |
| `stamp` | Sensor timestamp. |
| `frame_id` | Camera frame id from the image header. |

Detections outside the configured minimum and maximum distance are filtered out. The node can also publish an annotated image with marker outlines for RViz or debugging.

### 4.3 Semantic Perception

The `semantic_perception` node combines optional image classification with LiDAR clustering. If the `ultralytics` package is installed, it loads YOLOv8 weights and classifies RGB detections into mission-relevant labels such as `person` and `furniture`. If YOLO is unavailable, the node still runs in LiDAR-only mode, which keeps the rest of the stack operational.

The LiDAR path groups scan returns into clusters using range continuity. Clusters with person-sized widths become `person_candidate`, wider clusters become `furniture_candidate`, and smaller clusters become `small_obstacle`. The node also estimates `door_open` candidates by looking for gap-like structures framed by LiDAR returns. The output is a single JSON object on `/world/objects`, with detections in the robot base frame.

This perception layer is intentionally lightweight. Its purpose is not to produce a complete semantic map, but to provide enough short-term context for the executive to make better failure-recovery decisions.

### 4.4 World Model

The `world_model` node fuses recent semantic detections and ArUco detections into a time-decayed state snapshot. Different object classes have different half-lives:

| Class | Half-life | Rationale |
| --- | ---: | --- |
| `person`, `person_candidate` | 1 s | People move quickly, so detections should expire quickly. |
| `furniture` | 30 s | Furniture is approximately static over a demo. |
| `furniture_candidate` | 15 s | LiDAR-only furniture evidence is useful but less certain. |
| `door_open` | 3 s | Door/gap observations are useful but flickery. |
| `small_obstacle` | 5 s | Small obstacles should persist briefly. |
| `landmark` | 5 s | Landmark sightings should remain available long enough for arrival logic. |

The world model publishes both a JSON state snapshot and RViz markers. This gives the executive a consistent recent view and gives the operator a way to inspect what the robot believes it sees.

### 4.5 Tour Planning

The `tour_planner` node loads points of interest from `config/pois.yaml`. Each POI has an id, name, map-frame target pose, optional yaw, matching landmark id, and fallback description. The current POI coordinates in the repository are placeholders; for a real tour they should be replaced with poses from the saved REPF B4 map.

The planner builds a route with nearest-neighbor construction followed by 2-opt refinement. For the small number of POIs expected in the demo, this is simple, fast, and adequate. The route is not meant to solve a large traveling-salesperson problem exactly. Its purpose is to produce an efficient and understandable ordering over known tour stops.

The planner also supports dynamic behavior through two topics. A replan request provides a new start position and reason. A drop request permanently removes a POI from future plans for the current run. This allows the executive to skip unreachable stops or respond to a changed environment without restarting the entire ROS stack.

### 4.6 Executive State Machine

The `executive` node is the mission-level controller. It subscribes to the current plan, the world model, landmark detections, and AMCL pose. It publishes narration, status, replan requests, and drop requests. It also sends `NavigateToPose` goals to Nav2.

The executive has six states:

| State | Meaning |
| --- | --- |
| `IDLE` | Waiting for a plan. |
| `NAVIGATING` | A Nav2 goal is active for the current POI. |
| `AT_POI` | The POI has been reached by Nav2 success or landmark confirmation. |
| `NARRATING` | The robot is speaking or printing the POI description. |
| `RECOVERING` | The robot is waiting, replanning, or dropping a POI after a failure. |
| `DONE` | The tour is complete or all remaining POIs have been skipped. |

The executive can mark arrival in two ways. First, if Nav2 reports success, the robot transitions to `AT_POI`. Second, if the target POI has a matching landmark id and the detector sees that landmark within the configured arrival distance, the executive can cancel the active Nav2 goal and transition early. This makes the marker an independent symbolic confirmation of arrival.

### 4.7 Failure Classification

The most mission-specific part of the project is the executive's failure classifier. When Nav2 fails, the executive inspects the latest world snapshot and classifies the failure:

| Condition | Classification | Recovery |
| --- | --- | --- |
| Person or person candidate within the blocking distance | `PERSON_BLOCKING` | Ask politely to pass, wait, then retry. |
| Newly observed furniture-class object | `FURNITURE_MOVED` | Narrate that the environment changed and request a replan. |
| No forward `door_open` evidence after retries are exhausted | `DOOR_CLOSED` | Drop the current POI, narrate the skip, and replan. |
| Retries exhausted with no other explanation | `POI_UNREACHABLE` | Drop the current POI and continue. |
| No specific explanation | `UNKNOWN` | Wait briefly and retry. |

This is a deliberately small classifier, but it demonstrates the advantage of using perception and world state above Nav2. The robot does not need to know every cause of failure. It only needs enough context to choose a more appropriate response than repeatedly sending the same failed goal.

### 4.8 Narration

The `narrator` node subscribes to `/tour/narration`, prints messages in the terminal, and optionally speaks them with `pyttsx3`. This separation keeps text generation in the executive or announcer and speech output in one reusable node.

For the perception-only demo path, `landmark_announcer` replaces the executive. It subscribes directly to `/landmarks/detected`, waits until a marker has remained close enough for a dwell period, enforces a cooldown, and publishes the matching description. This makes the landmark-recognition component reusable by other teams even if they do not adopt the full tour planner.

### 4.9 Reactive Safety Monitor

The `safety_monitor` node watches the forward LiDAR arc for ranges below a configurable stop distance. When tripped, it publishes a zero `TwistStamped` to `/cmd_vel` for a short hold period and publishes a warning on `/safety/event`. It is intended to run on the robot for low latency. The monitor does not replace Nav2's obstacle avoidance; it provides an additional reflex for close-range obstacles during demonstration.

## 5. Implementation

The implementation is organized as a single ROS 2 package under `ros2_ws/src/robot_tour_guide`. The package uses standard ROS 2 Python entry points for all nodes and installs launch and configuration files into the package share directory.

The main launch files are:

| Launch file | Purpose |
| --- | --- |
| `tour_guide.launch.py` | Starts the full tour-guide stack on top of existing TurtleBot 4 bringup and Nav2. |
| `tour_guide_sim.launch.py` | Wraps the full stack for a Gazebo/TurtleBot 4 simulation workflow. |
| `perception_only.launch.py` | Starts ArUco detection, semantic perception, world model, landmark announcer, and narrator without Nav2. |

The main configuration files are:

| File | Purpose |
| --- | --- |
| `params.yaml` | Centralized ROS parameters for all nodes. |
| `pois.yaml` | Tour stop names, poses, landmark ids, and fallback descriptions. |
| `landmarks.yaml` | Marker id to landmark name and narration text. |

The package favors inspectable data and simple interfaces. Each JSON topic can be echoed during debugging, and most parameters can be changed from YAML without editing source code. This is useful in a robot demo setting where camera topics, marker sizes, map poses, and detection distances may need adjustment.

## 6. Evaluation Methodology

The project is intended to be evaluated in two stages.

### 6.1 Perception-Only Evaluation

The perception-only evaluation tests whether printed ArUco markers can trigger correct descriptions without autonomous navigation. The robot is manually driven with teleoperation. For each marker, the tester records whether the marker is detected, whether the correct narration is published, the approximate detection distance, and whether repeated announcements are prevented by the cooldown.

Recommended measurements:

| Measurement | Description |
| --- | --- |
| Detection success rate | Number of successful detections divided by marker presentations. |
| Correct-announcement rate | Number of correct narrations divided by successful detections. |
| Detection range | Approximate distance where the marker is reliably recognized. |
| False announcement count | Number of incorrect or repeated announcements. |

### 6.2 Full-Tour Evaluation

The full-tour evaluation tests the integrated planner, executive, Nav2 interface, landmark confirmation, narration, and recovery logic. The robot begins from a known pose in the mapped environment and attempts to visit every POI in the current plan. During the run, the tester records plan order, navigation success, landmark confirmation, narration success, failure class, recovery action, and final tour completion.

Recommended measurements:

| Measurement | Description |
| --- | --- |
| Tour completion rate | Percentage of runs that end in `DONE`. |
| POI visit rate | Percentage of POIs reached or correctly skipped. |
| Mean time per POI | Average time from dispatch to arrival or skip. |
| Recovery accuracy | Whether the classified failure matched the observed situation. |
| Human-facing behavior | Whether narration and polite wait behavior occurred at the right time. |

### 6.3 Implementation Verification Performed

The repository was inspected and a Python syntax check was performed across the eight node implementation files. The syntax check passed for:

| File |
| --- |
| `aruco_detector.py` |
| `semantic_perception.py` |
| `world_model.py` |
| `tour_planner.py` |
| `executive.py` |
| `narrator.py` |
| `landmark_announcer.py` |
| `safety_monitor.py` |

No stored robot-run logs, simulation bags, or quantitative experiment results were present in the repository at the time this report was prepared. Therefore, the results below distinguish between verified implementation properties and experimental results that should be filled in from the live demonstration or recorded trials.

## 7. Results

### 7.1 Implementation Results

The project produced a complete ROS 2 package with all major tour-guide components represented as runnable nodes. The package has a centralized configuration file, packaged launch files, and separate demo modes for full autonomy and ArUco-only narration. This satisfies the main software-design goal of creating a mission-level tour-guide stack rather than a single isolated perception or navigation node.

The ArUco path is modular. `aruco_detector` can be used independently from the rest of the stack, and `landmark_announcer` can convert marker detections into narration without the executive. This supports the code-adoption goal because other teams could reuse the symbol-to-description behavior without adopting the full planner.

The planner and executive implement the expected deliberative and hybrid behavior. The planner computes a route over POIs rather than requiring a hard-coded visit order. The executive dispatches goals, handles state transitions, confirms arrival through landmarks, and selects recoveries based on perceived conditions.

The world model provides a useful intermediate representation. Rather than having the executive subscribe directly to every raw sensor and perception signal, it reads a compact recent snapshot containing objects, landmarks, and counts. This makes failure classification easier to understand and modify.

### 7.2 Expected Demonstration Results

The expected behavior in the quick ArUco-only demo is:

| Scenario | Expected result |
| --- | --- |
| Marker is visible within configured range for the dwell time | Correct landmark description is published to `/tour/narration`. |
| Robot remains facing the same marker | Cooldown prevents repeated narration spam. |
| Robot turns to a different known marker | New landmark description is published. |
| Unknown marker is seen | Unknown-marker narration is published or logged. |

The expected behavior in the full tour demo is:

| Scenario | Expected result |
| --- | --- |
| Nav2 reaches a POI normally | Executive transitions from `NAVIGATING` to `AT_POI`, then `NARRATING`. |
| Matching landmark is seen before Nav2 success | Executive confirms arrival early and narrates the landmark. |
| Person blocks the path | Executive classifies `PERSON_BLOCKING`, asks to pass, waits, and retries. |
| Furniture appears in the path | Executive classifies `FURNITURE_MOVED`, requests a replan, and retries with a new plan. |
| POI remains unreachable after retries | Executive drops the POI, narrates the skip, and continues. |
| All POIs are visited or skipped | Executive transitions to `DONE` and announces tour completion. |

### 7.3 Results To Insert From Final Runs

The following table should be completed using data from the final real-robot or simulation runs:

| Run | Mode | POIs attempted | POIs reached | POIs skipped | Recoveries triggered | Tour completed? | Notes |
| --- | --- | ---: | ---: | ---: | --- | --- | --- |
| 1 | ArUco-only | N/A | N/A | N/A | N/A | N/A | Fill in marker detection and narration observations. |
| 2 | Full tour | TBD | TBD | TBD | TBD | TBD | Fill in after Nav2 demo. |
| 3 | Full tour with obstacle | TBD | TBD | TBD | TBD | TBD | Fill in after person/furniture/door scenario. |

## 8. Discussion

The main strength of the project is that it treats the tour-guide mission as more than navigation. The robot needs a plan, but it also needs evidence that a place has been reached, a way to communicate with visitors, and a policy for responding to common failures. The architecture captures these requirements with relatively simple components.

The ArUco-based landmark design is especially appropriate for the assignment environment. A printed marker is easy to set up, easy to explain during a demo, and much more reliable than open-ended visual recognition in a classroom. It also makes narration data-driven: changing a landmark description requires editing YAML rather than changing source code.

The failure classifier is useful because it introduces mission-level interpretation. Nav2 can report that navigation failed, but the tour guide benefits from asking why the failure may have happened. A person blocking the path should not be treated the same way as a permanently unreachable POI. Similarly, moved furniture suggests replanning, while a closed door or blocked stop may justify skipping a POI.

The system also has limitations. First, semantic perception is approximate. YOLO detections are image-space observations, while LiDAR clusters are geometric observations in the robot frame; the current fusion is simple and does not perform full object tracking or camera-LiDAR association. Second, the door-open detector is heuristic and may not generalize beyond simple gap-like structures. Third, the POI coordinates in the repository are placeholders, so the full tour depends on careful map creation and waypoint entry before demonstration. Fourth, the planner tracks dropped POIs but not completed POIs across replans; as currently written, a replan can reintroduce previously visited stops unless the executive or planner is extended to track visited POIs.

Another practical limitation is that the repository does not include recorded experiment data. For a final technical report, live-trial measurements should be added so the results section contains actual completion rates, detection rates, and recovery outcomes. Without those measurements, conclusions about real-world reliability must remain cautious.

## 9. Conclusions

This project demonstrates that a hybrid architecture is a strong fit for a TurtleBot 4 tour-guide mission. The deliberative planner gives the robot a route through named points of interest. The executive connects that route to Nav2, narration, landmark confirmation, and recovery. The perception and world-model layers provide enough environmental context for the executive to choose different responses to different failure types. The reactive safety monitor provides a separate short-latency stop behavior for close obstacles.

The most important conclusion is that mission-level behavior can be improved without replacing the existing navigation stack. Nav2 remains responsible for path planning and control, while the tour-guide package adds symbolic context and task-level decision-making. This makes the system realistic for a course project and reusable by other teams.

The second conclusion is that simple symbolic landmarks are valuable in constrained indoor robot missions. ArUco markers make location recognition reliable, data-driven, and demonstrable. They also provide a clean bridge between perception and narration.

Finally, the project shows that carefully scoped recovery logic can make a robot appear more intelligent. Even a small set of failure classes can produce behavior that is easier for observers to understand than blind retrying.

## 10. Future Work

The most immediate future work is to collect and include quantitative real-robot data. The report should be updated with detection success rates, route completion rates, recovery outcomes, and timing statistics from the final demonstration.

The planner should be extended to track visited POIs so replanning cannot return the robot to stops that have already been completed. The executive should also ignore stale Nav2 cancellation results after early landmark arrival, because a cancelled goal should not trigger recovery once the robot has already transitioned to `AT_POI`.

The perception layer could be improved with more explicit sensor fusion. Camera detections could be associated with LiDAR clusters, tracked over time, and transformed into the map frame. This would make person and furniture classification more reliable and would support better explanations of navigation failures.

The door model could also be improved. Instead of relying on short-term gap detection, the system could represent known door locations from the map or from a semantic environment file and reason about whether the expected doorway is currently passable.

Finally, the system could support richer tour interaction. Examples include visitor questions, selectable tour themes, dynamic route changes based on time remaining, and a web or RViz panel showing the current stop, next stop, and reason for any recovery behavior.

## Bibliography

Brooks, R. A. (1986). A robust layered control system for a mobile robot. *IEEE Journal on Robotics and Automation, 2*(1), 14-23. https://doi.org/10.1109/JRA.1986.1087032

Garrido-Jurado, S., Munoz-Salinas, R., Madrid-Cuevas, F. J., & Marin-Jimenez, M. J. (2014). Automatic generation and detection of highly reliable fiducial markers under occlusion. *Pattern Recognition, 47*(6), 2280-2292. https://doi.org/10.1016/j.patcog.2014.01.005

OpenCV. (2026). Detection of ArUco markers. https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

Open Robotics. (2024). ROS 2 Jazzy Jalisco release documentation. https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html

Open Navigation LLC. (2026). NavigateToPose action documentation. https://api.nav2.org/actions/humble/navigatetopose.html

TurtleBot 4 Project. (2026). TurtleBot 4 user manual: Sensors. https://turtlebot.github.io/turtlebot4-user-manual/software/sensors.html

Ultralytics. (2026). Ultralytics YOLO documentation and software. https://docs.ultralytics.com/

## Appendix A. Source Code and Launch File Documentation

### A.1 Node Implementations

| File | Description |
| --- | --- |
| `robot_tour_guide/aruco_detector.py` | ROS 2 node for detecting ArUco markers from the OAK-D RGB image, estimating marker pose, filtering detections by distance, and publishing JSON detections and annotated images. |
| `robot_tour_guide/semantic_perception.py` | ROS 2 node for optional YOLO-based object classification and LiDAR cluster classification into people, furniture candidates, small obstacles, and door-open hints. |
| `robot_tour_guide/world_model.py` | ROS 2 node that time-decays recent semantic and landmark detections and publishes a fused JSON state plus RViz markers. |
| `robot_tour_guide/tour_planner.py` | ROS 2 node that loads POIs from YAML, computes a nearest-neighbor plus 2-opt tour, republishes the current plan, and supports replanning and dropped POIs. |
| `robot_tour_guide/executive.py` | ROS 2 node implementing the mission-level state machine, Nav2 action interface, landmark arrival confirmation, narration, and recovery classification. |
| `robot_tour_guide/landmark_announcer.py` | ROS 2 node for the perception-only demo path, mapping marker detections directly to narration with dwell and cooldown logic. |
| `robot_tour_guide/narrator.py` | ROS 2 node that prints narration and optionally speaks it with text-to-speech. |
| `robot_tour_guide/safety_monitor.py` | ROS 2 node that monitors the forward LiDAR arc and publishes zero velocity when a close obstacle is detected. |

### A.2 Launch Files

| File | Description |
| --- | --- |
| `launch/tour_guide.launch.py` | Full tour-guide stack for use with already-running TurtleBot 4 bringup and Nav2. |
| `launch/tour_guide_sim.launch.py` | Simulation wrapper for running the same tour-guide stack with Gazebo/TurtleBot 4 simulation. |
| `launch/perception_only.launch.py` | Lower-risk demo path using ArUco detection, world model, landmark announcer, and narrator without autonomous navigation. |

### A.3 Configuration Files

| File | Description |
| --- | --- |
| `config/params.yaml` | Central parameter file for camera topics, marker size, semantic perception thresholds, world-model decay, planner behavior, executive timing, narration, and safety monitor settings. |
| `config/pois.yaml` | YAML database of POI ids, names, map poses, landmark ids, and fallback descriptions. Current coordinates are placeholders and should be replaced with real mapped poses before a full tour demo. |
| `config/landmarks.yaml` | YAML database mapping ArUco marker ids to landmark names and narration descriptions. |

## Appendix B. Installation and Running Instructions

### B.1 Build

```bash
cd Robot-Tour-Guide/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Optional dependencies:

```bash
pip install --user ultralytics pyttsx3
```

### B.2 Perception-Only Demo

```bash
ros2 launch robot_tour_guide perception_only.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Drive the robot within approximately 1.5 m of a printed marker and face the marker for the configured dwell time. The corresponding landmark description should be published to `/tour/narration`.

### B.3 Full Tour Demo

Before running the full tour, create or load a map and replace the placeholder coordinates in `config/pois.yaml` with real map-frame POI poses.

```bash
ros2 launch robot_tour_guide tour_guide.launch.py
```

Nav2, localization or SLAM, TurtleBot 4 bringup, the OAK-D camera, and the LiDAR should already be running.

## Appendix C. Team Member Contributions

This appendix should be completed with the actual team member names before submission.

| Team member | Design contributions | Implementation contributions | Testing contributions | Reporting contributions |
| --- | --- | --- | --- | --- |
| Team member 1 | Hybrid architecture, node responsibilities, recovery policy. | Fill in specific files or features. | Fill in tested scenarios. | Fill in report sections. |
| Team member 2 | Fill in. | Fill in. | Fill in. | Fill in. |
| Team member 3 | Fill in. | Fill in. | Fill in. | Fill in. |

## Appendix D. Known Issues and Report Completion Checklist

The following items should be resolved or acknowledged before final submission:

| Item | Status |
| --- | --- |
| Replace author placeholder with actual names. | TODO |
| Replace placeholder POI coordinates in `pois.yaml` with real map poses. | TODO |
| Add real demonstration results to Section 7.3. | TODO |
| Confirm whether the repository license should be MIT or Apache 2.0 and make README, `package.xml`, `setup.py`, and `LICENSE` consistent. | TODO |
| Consider tracking visited POIs during replanning. | TODO |
| Consider ignoring stale Nav2 cancellation results after landmark-confirmed arrival. | TODO |

