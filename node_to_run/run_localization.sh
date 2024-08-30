#!/bin/bash
source install/setup.bash
colcon build --packages-select localization && ros2 run localization localization

