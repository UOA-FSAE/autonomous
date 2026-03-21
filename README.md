# UOA FSAE — Autonomous System

ROS 2 workspace for the University of Auckland Formula SAE autonomous go-kart.
Currently, the car drives itself around a cone-lined track using a ZED stereo camera for perception,
Kalman-filter-based landmark mapping, fast-tube path planning, and Stanley lateral control,
all bridged to the physical kart over CAN.

> **Platform:** NVIDIA Jetson (L4T 35.4) · ROS 2 Jazzy (on the autonomous PC, Humble on the Jetson - needs to be upgraded) · ZED 2i stereo camera
> **Build system:** colcon (ament_cmake + ament_python)

---

## Repository Layout

```
autonomous/
├── src/
│   ├── fsae_interfaces/        # Shared ROS msg/srv definitions (ament_cmake)
│   ├── fsae_control/           # Stanley lateral controller (ament_python)
│   ├── fsae_planning/          # Path planning — fast-tube & centerline (ament_python)
│   ├── fsae_perception/
│   │   ├── zed_perception/     #   ZED camera + YOLO cone detection (C++, TensorRT)
│   │   └── fsae_slam/          #   Cone landmark SLAM (C++, skeleton — see below)
│   ├── gocart/
│   │   ├── gocart_bringup/     #   Main launch files (autonomous, scrutineering)
│   │   ├── gocart_control/     #   Ackermann-to-CAN bridge, teleop, mock stimulus
│   │   ├── gocart_driver/      #   CAN decoder for Jetson Nano
│   │   ├── gocart_description/ #   URDF model + meshes
│   │   └── CanTalk/            #   Serial CAN adapter driver (submodule)
│   ├── scrutineering/          # FSAE technical inspection mission controller
│   ├── visualisation/
│   │   ├── base_tf/            #   Static TF broadcaster
│   │   ├── cone_map_foxglove_visualiser/
│   │   ├── path_planning_visualiser/
│   │   └── pure_pursuit_visualiser/
│   └── third_party/
│       ├── foxglove-bridge/    #   WebSocket bridge for Foxglove Studio
│       └── yolov7/             #   YOLO model training reference
├── Formula-Student-Driverless-Simulator/   # FS Driverless Sim (submodule)
├── .devcontainer/              # Docker dev environment (Jetson + ZED containers)
├── makefile                    # Dev environment build helper
├── mock_params.yaml            # Parameter file for mock controller testing
└── requirements/               # pip dependency lists
```

---

## Package Reference

### Core Pipeline

| Package | Type | Description | Executables |
|---------|------|-------------|-------------|
| **fsae_interfaces** | cmake | All custom `.msg` and `.srv` definitions | *(msg/srv generation only)* |
| **zed_perception** | cmake | ZED camera node: stereo capture → YOLO (TensorRT) → cone detections + localisation | `zed_launch_node`, `cone_subscriber`, `localisation_subscriber` |
| **fsae_slam** | cmake | Cone landmark SLAM (C++ skeleton — porting from Python reference) | `cone_landmark_mapper` |
| **fsae_planning** | python | Path planning algorithms | `fasttube`, `fasttube_without_kalman`, `centerline_planner`, `center_line` |
| **fsae_control** | python | Stanley lateral controller | `controller` |

### Go-Kart Hardware

| Package | Type | Description | Executables |
|---------|------|-------------|-------------|
| **CanTalk** | python | Serial CAN adapter driver (candapter) | `candapter_node` |
| **gocart_control** | python | Ackermann-to-CAN, system status, teleop, mock stimulus | `ack_to_can_node`, `as_status_node`, `trajectory_follower`, `joystick_teleop`, `mock_stimulus` |
| **gocart_driver** | python | Jetson Nano CAN bus decoder | `can_decoder_jnano` |
| **gocart_description** | python | URDF model and STL meshes | *(data only)* |
| **gocart_bringup** | python | Launch files for the full autonomous system | *(launch only)* |

### Events & Visualisation

| Package | Type | Description | Executables |
|---------|------|-------------|-------------|
| **scrutineering** | python | FSAE technical inspection mission | `inspection_mission_node`, `test_node` |
| **base_tf** | python | Static TF broadcaster for kart frame | `base_tf` |
| **cone_map_foxglove_visualiser** | python | Publishes cone map markers for Foxglove | `visualiser` |
| **path_planning_visualiser** | python | Trajectory & image throttle visualisation | `visualiser`, `visualiser2`, `image_throttler` |
| **pure_pursuit_visualiser** | python | Pure pursuit debug visualisation | `visualiser` |

---

## Message Definitions (fsae_interfaces)

<details>
<summary>Messages (17)</summary>

| Message | Key Fields |
|---------|------------|
| `Detections` | `Pose car_pose`, `Point[] yellow`, `Point[] blue`, `Point[] small_orange`, `Point[] big_orange` |
| `Track` | `Point[] cones` |
| `Cone` | `uint32 id`, `float32 confidence`, `uint8 colour`, `PoseWithCovariance pose` |
| `ConeMap` / `ConeMapStamped` | `Cone[] cones` (± header) |
| `ConeStamped` | `Header`, `Cone` |
| `CAN` / `CANStamped` | `uint16 id`, `bool is_rtr`, `uint8[8] data` (± header) |
| `HardwareStates` / `HardwareStatesStamped` | EBS, TS, gear, master switch, ASB, brakes (± header) |
| `MissionStates` / `MissionStatesStamped` | `mission_selected`, `mission_finished` (± header) |
| `AllStates` | `uint16 id`, `AckermannDrive[] states` |
| `AllTrajectories` | `uint16 id`, `PoseArray[] trajectories` |
| `BoundaryStamped` | `Header`, `float32[] coords` |
| `OccupancyGrid` | `Header`, `uint8[] occupancy_grid`, width, height, resolution |
| `Pulse` | `bool value`, `string event_type` |

</details>

<details>
<summary>Services (1)</summary>

| Service | Request | Response |
|---------|---------|----------|
| `CANSendReq` | `CAN can` | `bool sent` |

</details>

---

## Launch Files

### `gocart_autonomous.launch.py` — Full Autonomous Pipeline

The primary launch for competition runs. Namespace: `moa`.

```
ZED Camera → Cone Landmark Mapper → Fast-Tube Planner → Stanley Controller → CAN Adapter → Ackermann-to-CAN
```

| # | Package | Executable | Node Name | Key Params |
|---|---------|------------|-----------|------------|
| 1 | zed_perception | zed_launch_node | perception | — |
| 2 | fsae_slam | cone_landmark_mapper | cone_landmark_mapper | `invert_cones: true` |
| 3 | fsae_planning | fasttube | centerline_planner | `invert_cones: true` |
| 4 | fsae_control | controller | stanley_controller | `vel: 9.0` |
| 5 | CanTalk | candapter_node | — | — |
| 6 | gocart_control | ack_to_can_node | ack_to_can | `can_id: 0x300` |

Optional (set `visualize = True`): image_throttler + foxglove_bridge on port 8765.

### `gocart_autonomous_no_kalman.launch.py` — Without Kalman Filter for cone positions

Same pipeline but skips `cone_landmark_mapper`, uses `fasttube_without_kalman` instead,
runs at `vel: 10.0`, and always enables Foxglove visualisation.

### Other Launch Files

| Launch File | Location | Purpose |
|-------------|----------|---------|
| `base.launch.py` | gocart_bringup | Base nodes: ack_to_can + sys_status + candapter |
| `scrutineering.launch.py` | gocart_bringup | base.launch + inspection mission node |
| `camera_stimulus_test_launch.py` | gocart_control | Full pipeline with mock CAN stimulus (bench testing) |
| `motec_mock_controller_test_launch.py` | gocart_control | CAN adapter + mock stimulus only |
| `gocart_driver.launch.py` | gocart_driver | CAN decoder node |
| `fsae_slam.launch.py` | fsae_slam | SLAM node standalone |
| `zed_perception.launch.py` | zed_perception | ZED camera node standalone |
| `path_planning.launch.py` | fsae_planning | Dynamic planner selection via `node_name` arg |
| `pure_pursuit_visualiser_launch.py` | pure_pursuit_visualiser | Pure pursuit visualisation |
| `urdf_model.py` | gocart_description | Spawn URDF in Gazebo |

---

## Architecture

```
                    ┌──────────────┐
                    │  ZED Camera  │
                    │  (stereo +   │
                    │   YOLO)      │
                    └──────┬───────┘
                           │ /cone_detection (Detections)
                           │ /car_position   (Pose)
                           ▼
                    ┌──────────────┐
                    │  fsae_slam   │
                    │  (landmark   │
                    │   mapper)    │
                    └──────┬───────┘
                           │ /left_track  (Track)
                           │ /right_track (Track)
                           ▼
                    ┌──────────────┐
                    │ fsae_planning│
                    │  (fast-tube) │
                    └──────┬───────┘
                           │ /selected_trajectory
                           ▼
                    ┌──────────────┐
                    │ fsae_control │
                    │  (Stanley)   │
                    └──────┬───────┘
                           │ /cmd_vel (AckermannDrive)
                           ▼
              ┌────────────┴────────────┐
              │    gocart_control       │
              │  (ack_to_can_node)      │
              └────────────┬────────────┘
                           │ /can (CAN)
                           ▼
              ┌────────────┴────────────┐
              │       CanTalk           │
              │   (candapter_node)      │
              └────────────┬────────────┘
                           │ Serial USB
                           ▼
                     Physical Kart
```

---

## Building

```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build all packages (skip those needing ZED SDK / external deps)
colcon build --packages-skip zed_perception foxglove_bridge

# Source the workspace
source install/setup.bash
```

> **Note:** `zed_perception` requires the ZED SDK 4, CUDA, and TensorRT — build it on
> the Jetson or a machine with the SDK installed. `foxglove_bridge` is a third-party
> submodule that builds separately.

---

## Running

```bash
# Full autonomous pipeline
ros2 launch gocart_bringup gocart_autonomous.launch.py

# Without SLAM (direct cone-to-path, with visualisation)
ros2 launch gocart_bringup gocart_autonomous_no_kalman.launch.py

# Bench test with mock CAN stimulus
ros2 launch gocart_control camera_stimulus_test_launch.py

# Scrutineering inspection
ros2 launch gocart_bringup scrutineering.launch.py
```

---

## Submodules

| Submodule | Path | Source |
|-----------|------|--------|
| CanTalk | `src/gocart/CanTalk` | [UOA-FSAE/CanTalk](https://github.com/UOA-FSAE/CanTalk) |
| foxglove-bridge | `src/third_party/foxglove-bridge` | [foxglove/ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge) |
| yolov7 | `src/third_party/yolov7` | [WongKinYiu/yolov7](https://github.com/WongKinYiu/yolov7) |
| FS Driverless Simulator | `Formula-Student-Driverless-Simulator` | [FS-Driverless/Formula-Student-Driverless-Simulator](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator) |

---
