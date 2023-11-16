FROM stereolabs/zed:4.0-tools-devel-l4t-r35.4
LABEL Name=zed_sdk Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY ../ /

# setup sources.list and keys for ROS
RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

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

# install ros2 packages
RUN mkdir /ws/src/ && cd "$_" && \
    git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    cd .. && \
    source /opt/ros/humble/setup.bash && \ 
    rosdep update && \
    rosdep install --from-paths src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy && \
    colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ --base-paths src \
        --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
        ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
        ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' && \
    rm -rf /var/lib/apt/lists/*

# doesnt copy to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \ 
    echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]