# This startup script aims to setup the entire perception system environment so 

#!/bin/bash
pip3 install requests
python3 /usr/local/zed/get_python_api.py
cd /ws/src/perception/cone-detection/cone_detection
git clone git@github.com:WongKinYiu/yolov7.git
cd yolov7
pip3 install -r requirements.txt
cd /ws
pip3 install pyopengl

sudo apt-get install ros-humble-ackermann-msgs

source /opt/ros/humble/setup.bash

# cd /ws/src/hardware_drivers
# git clone git@github.com:foxglove/ros-foxglove-bridge.git
# cd /ws
