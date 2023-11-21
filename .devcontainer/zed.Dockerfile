FROM stereolabs/zed:4.0-tools-devel-l4t-r35.4
LABEL Name=zed_sdk Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt update && apt install -y gnupg wget software-properties-common && \
    add-apt-repository universe

RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | \
    apt-key add - && \
    echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | \
    tee -a "/etc/apt/sources.list"

RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup timezone & install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    tzdata \
    dirmngr \
    gnupg2 \
    git \
    ros-humble-ros-base \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

ENV ROS_DISTRO humble

# setup colcon mixin and metadata
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

COPY ./src/moa/cone-detection /ws/src/moa/cone-detection
COPY ./src/moa/moa_description /ws/src/moa/moa_description

# install ros2 packages
RUN cd /ws/src/ && \
    git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    cd .. && \
    source /opt/ros/humble/setup.bash && \ 
    rosdep update && apt-get update && \
    rosdep install --from-paths src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN cd /usr/local/zed && \
    pip install requests && \
    python3 get_python_api.py

RUN colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ --base-paths src \
        --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
        ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
        ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \ 
    echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]