FROM osrf/ros:humble-desktop
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY . .

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src && \
    colcon build

RUN apt-get update && apt-get install -y \
    python3-pip && \
    apt-get clean
    
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
