FROM osrf/ros:humble-desktop
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    ros-humble-foxglove-bridge && \ 
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

ENV ROS_DISTRO humble

COPY . .

RUN source /opt/ros/humble/setup.bash && \
    rosdep update --rosdistro $ROS_DISTRO && apt-get update && \
    rosdep install --from-paths src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy && \ 
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN source /opt/ros/humble/setup.bash && \
    colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ --base-paths src \
        --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
        ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
        ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD [ "bash" ]