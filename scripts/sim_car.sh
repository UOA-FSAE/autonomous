#!/bin/bash
ros2 topic pub --once /race_controller/create std_msgs/String "data: test"
ros2 topic pub --once /test/cmd_throttle std_msgs/Float32 "data: 3.0"