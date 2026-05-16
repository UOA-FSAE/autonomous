# TF Tree Setup Guide for Trolly with ZED + Velodyne

## Problem
Your sensor frames aren't connected to the robot base_link, so RViz can't find the transform chain:
- Velodyne publishes `frame_id: "velodyne"` but that frame doesn't exist in TF tree
- ZED camera doesn't publish frame_ids or TF
- Missing: `map` -> `base_link` -> `zed_camera_optical_frame` and `velodyne`

## Solution

### Terminal 1: Launch the trolly description with transforms
```bash
ros2 launch trolly_description trolly_with_sensors.launch.py
```

This publishes:
- `robot_state_publisher`: Publishes the URDF transforms (base_link -> camera_link, base_link -> velodyne, etc.)
- `static_transform_publisher` (map->base_link): Connects your map frame to the robot base
- `static_transform_publisher` (base_link->zed_camera_optical_frame): Backup in case ZED doesn't publish frame_id

### Terminal 2: Launch ZED perception
```bash
ros2 launch zed_perception zed_perception.launch.py
```

### Terminal 3: Launch Velodyne driver
```bash
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
```

### Terminal 4: Launch velodyne laserscan converter
```bash
ros2 launch velodyne_laserscan velodyne_laserscan_node-launch.py
```

### Terminal 5: RViz2
```bash
rviz2
```

## In RViz2

1. **Set Fixed Frame to `map`** (top-left dropdown)
2. **Add > TF** to visualize the transform tree
3. **Add > LaserScan** and set Topic to `/scan` (from velodyne_laserscan)
4. **Add > Image** and set Topic to `/zed/image` (from zed_perception)

## What the transforms look like
```
map (root frame)
├── base_link (robot center)
│   ├── camera_link
│   │   └── zed_camera_optical_frame (ZED sensor frame)
│   ├── velodyne (Velodyne lidar frame)
│   └── (other joints from trolly URDF)
```

## Next Steps

### (Optional) Fix ZED frame_id in code
If the ZED image still doesn't have correct frame_ids, add this to `zed_launch.cpp`:
```cpp
// In the cone_detection_loop() function where image_msg is populated:
image_msg.header.frame_id = "zed_camera_optical_frame";

// In car_position loop:
position_msg.header.frame_id = "base_link";
```

### (Optional) Remap velodyne frame_id at launch time
Edit the velodyne params or use ros2 remapping:
```bash
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py --ros-args --remap velodyne:=velodyne_points
```

### (Production) Replace static map->base_link with SLAM
Once your SLAM system (fsae_slam) is publishing the map->base_link transform, the static publisher will be overridden automatically.
