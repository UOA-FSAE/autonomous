# ZED SDK Architecture Decision

## Decision: Use the ZED SDK directly, not the official ROS2 wrapper

## Why

Our cone detection pipeline requires a **bidirectional round-trip** with the ZED SDK that the official wrapper cannot provide.

The pipeline works like this:

1. `zed.grab()` — capture stereo frame + compute depth internally
2. `zed.retrieveImage()` — get left RGB image
3. Run YOLO (our TensorRT engine) — produces 2D bounding boxes
4. `zed.ingestCustomBoxObjects()` — feed those 2D boxes **back into** the SDK
5. `zed.retrieveObjects()` — SDK returns 3D positions, persistent tracking IDs, velocity, confidence

Steps 4 and 5 are the blocker. The official wrapper is a **one-way data source** — it publishes camera data as ROS topics but doesn't accept external detection inputs. There is no ROS topic or service to call `ingestCustomBoxObjects()` on the wrapper's internal camera instance.

### What about the wrapper's CUSTOM_YOLOLIKE_BOX_OBJECTS mode?

This mode exists in the wrapper config, but it expects you to provide an **ONNX file that the SDK runs internally** — not for you to run your own TensorRT engine and feed boxes back in. We'd lose control over our TensorRT optimizations, NMS tuning, and distance-tiered confidence filtering.

### What about subscribing to the depth map topic and doing manual lookups?

This works but loses everything `retrieveObjects()` does for free:

- **EKF tracking** — persistent cone IDs across frames, temporal smoothing
- **Smart depth sampling** — samples across the bounding box region, not just the center pixel
- **Outlier rejection** — handles stereo matching failures, NaN pixels, background bleed
- **Temporally-smoothed confidence** — our distance-tiered filter (60%/40%/25%) depends on this
- **Velocity estimation** per tracked object

A raw depth lookup at the center pixel of each bounding box is noisy and has no temporal consistency.

### What other FSAE teams do

Most competitive FSAE-D teams (AMZ/ETH Zurich, MIT Driverless, eufs/Edinburgh) use the ZED SDK directly for the same reason. The wrapper is designed for general robotics where modularity matters more than latency. Direct SDK is standard for custom detection + stereo fusion pipelines.

### Performance

The pipeline runs in a single process on the Jetson. Going through ROS topics would add ~16-24MB of unnecessary memcpy per frame (serialize image out, deserialize on subscriber, convert back to SDK format for fusion). On the Jetson's limited memory bandwidth, this matters.

## What we borrow from the wrapper

Even though we don't use the wrapper, we should follow its conventions:

- [ ] TF broadcasting (camera_link, odom, map frames)
- [ ] Proper error handling for camera disconnects/reconnection
- [ ] YAML-based parameterization (currently thresholds are hardcoded `#define`s)

## Competition-readiness issues (perception)

| Priority | Issue | Fix |
|----------|-------|-----|
| 1 | Detection loop measured at ~10 FPS (need 30+). Visualization always-on is likely bottleneck. | Set `visualisation = false` for competition, add FPS instrumentation |
| 2 | Six `std::cout` debug prints per frame, some in per-object loops | Remove or gate behind debug flag |
| 3 | Only publishes when both blue AND yellow cones visible — silences output on hairpins | Remove gate, always publish |
| 4 | No timestamps on Detections messages — planner can't compensate for latency at speed | Add Header to msg definition |
| 5 | All thresholds hardcoded as `#define`s | Convert to ROS parameters for trackside tuning |
| 6 | Velocity thread integrates raw IMU (known broken, comment in code) | Use `cam_w_pose.twist` from SDK instead |
| 7 | Motion blur at high speed with default auto-exposure | Cap exposure time, test at track |
