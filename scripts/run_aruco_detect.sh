#!/bin/bash
source install/setup.bash

colcon build --packages-select aruco_detection && ros2 run aruco_detection aruco_detection

