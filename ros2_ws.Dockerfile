FROM osrf/ros:humble-desktop
FROM stereolabs/zed:3.8-devel-cuda11.7-ubuntu22.04
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY . .

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-foxglove-bridge

RUN source /opt/ros/humble/setup.bash && \
    rosdep update
    
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
