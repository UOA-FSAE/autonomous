#!/bin/bash
set -e

sudo apt update
rosdep install --from-paths src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy --skip-keys="point_cloud_transport_plugins draco_point_cloud_transport" 

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

exec "$@"