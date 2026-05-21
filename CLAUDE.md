# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 workspace for the University of Auckland Formula SAE autonomous go-kart. The car drives around a cone-lined track using a ZED stereo camera (and Velodyne LiDAR, in progress) for perception, Kalman-filter SLAM, fast-tube path planning, and Stanley lateral control, bridged to the physical kart over CAN.

**Platform:** NVIDIA Jetson (L4T 36.5) · ROS 2 Jazzy · ZED 2i stereo camera · Velodyne LiDAR
**Build system:** colcon (ament_cmake for C++/CUDA packages, ament_python for Python packages)

## Build Commands

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build all packages (skip ZED-dependent packages if no SDK installed)
colcon build --packages-skip zed_perception foxglove_bridge

# Build a single package
colcon build --packages-select <package_name>

# Build with dependencies (useful when changing fsae_interfaces)
colcon build --packages-up-to <package_name>

# Source the workspace overlay
source install/setup.bash
```

`zed_perception` requires ZED SDK 5, CUDA, and TensorRT — only builds on the Jetson or a machine with these installed. `fsae_interfaces` must be built first (or via `--packages-up-to`) since most packages depend on its generated message types.

## Running

```bash
# Full autonomous pipeline
ros2 launch gocart_bringup gocart_autonomous.launch.py

# Without SLAM (direct cone-to-path, always enables Foxglove viz)
ros2 launch gocart_bringup gocart_autonomous_no_kalman.launch.py

# Bench test with mock CAN stimulus
ros2 launch gocart_control camera_stimulus_test_launch.py

# FSAE scrutineering inspection
ros2 launch gocart_bringup scrutineering.launch.py

# Individual nodes
ros2 launch zed_perception zed_perception.launch.py
ros2 launch fsae_slam fsae_slam.launch.py
ros2 launch fsae_planning path_planning.launch.py node_name:=fasttube
```

## Testing

```bash
# Run all tests
colcon test

# Run tests for a single package
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Python packages have flake8/pep257/copyright lint tests. `gocart_control` has unit tests for the Ackermann node and system status. `fsae_planning` has a trajectory optimisation test.

## Architecture — Data Pipeline

```
ZED Camera ──► zed_perception (YOLO TensorRT) ──► fsae_slam (Kalman landmark mapper)
                 /cone_detection (Detections)         /left_track, /right_track (Track)
                 /car_position (Pose)                          │
                                                               ▼
                                                    fsae_planning (fast-tube)
                                                      /selected_trajectory
                                                               │
                                                               ▼
                                                    fsae_control (Stanley)
                                                      /cmd_vel (AckermannDrive)
                                                               │
                                                               ▼
                                              gocart_control (ack_to_can_node)
                                                        /can (CAN)
                                                               │
                                                               ▼
                                                CanTalk (candapter_node) ──► Serial USB ──► Kart
```

All autonomous nodes run under the `moa` namespace. The no-Kalman pipeline skips `fsae_slam` and uses `fasttube_without_kalman` directly.

## Key Package Details

- **fsae_interfaces** — All custom `.msg`/`.srv` definitions. Must rebuild this and downstream packages when message definitions change. The `Detections` message carries colour-sorted cone arrays (yellow, blue, small_orange, big_orange) plus car pose.
- **zed_perception** — C++/CUDA node. Two perception approaches: `zed_launch_node` (standalone, owns `sl::Camera` directly) and `wrapper_perception_node` (subscribes to ZED ROS 2 wrapper topics for image, depth, camera_info, and odom; runs YOLO on the image, back-projects detections to 3D using depth + intrinsics). Launch with `use_wrapper:=true` to switch. Model file: `cone_detection_model.engine` (TensorRT, architecture-specific — must be regenerated from `.onnx` per platform).
- **fsae_slam** — C++ Kalman-filter landmark mapper. Has a Python reference implementation at `reference_cone_landmark_mapper.py`. Subscribes to `Detections`, publishes `Track` messages for left/right track boundaries.
- **fsae_planning** — Python. `fasttube_planner` (primary), `fasttube_without_kalman`, `centerline_planner`, `simple_centerline_planner`. Selectable at launch via the `node_name` argument.
- **fsae_control** — Python Stanley lateral controller. Velocity param (default 9.0 m/s with Kalman, 10.0 without).
- **velodyne_*** — Velodyne LiDAR driver packages (driver, pointcloud, laserscan, msgs). Being integrated for ground removal / supplementary perception.
- **zed-ros2-wrapper** — Stereolabs' official ROS 2 wrapper (submodule). Used by `wrapper_perception_node` for depth-fused 3D object detection.

## Submodules

CanTalk (`src/gocart/CanTalk`, branch: nightly), foxglove-bridge, yolov7, Formula-Student-Driverless-Simulator. Initialize with `git submodule update --init --recursive`.

## Dev Environment

The `makefile` sets up Docker dev containers, auto-detecting GPU (Jetson vs desktop) and architecture (ARM vs x86). Run `make build` then `make start` to launch. The `.devcontainer/` Dockerfiles are generated from `.docker_templates/` — don't edit the generated files directly.

## Conventions

- C++ packages use `ament_cmake`; Python packages use `ament_python` with `setup.py` + `setup.cfg`.
- Launch files are Python (`*.launch.py`).
- The `invert_cones` parameter on SLAM and planning nodes swaps left/right track assignment (for clockwise vs counter-clockwise tracks).
- CAN message ID for Ackermann commands: `0x300`.
- Foxglove Studio is used for visualisation (WebSocket bridge on port 8765).
- Mock testing uses `mock_stimulus` node with params from `mock_params.yaml`.
