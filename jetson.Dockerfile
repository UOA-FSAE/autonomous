FROM osrf/ros:humble-desktop
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY . .

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    colcon build 

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		build-essential \
        cmake \
        git \
        python3-pip

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]

