FROM arm64v8/ros:humble
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY . .

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    ros-humble-foxglove-bridge && \ 
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && apt-get update
    
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
