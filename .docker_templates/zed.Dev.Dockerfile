FROM stereolabs/zed:4.0-devel-cuda12.1-ubuntu22.04
LABEL Name=zed_sdk Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws

# setup timezone
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup environment
ENV ROS_DISTRO humble

# setup sources.list and keys
RUN apt update
RUN apt install -y curl gnupg wget software-properties-common 
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null





# install general packages
RUN apt-get update 
RUN apt-get install -q -y --no-install-recommends \
    ros-humble-foxglove-bridge \
    tzdata \
    dirmngr \
    gnupg2 \
    git \
    ros-humble-ros-base \
    build-essential \
    python3-pip\
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool 
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean

# setup colcon mixin and metadata
RUN rosdep init 
RUN rosdep update --rosdistro $ROS_DISTRO 
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 
RUN colcon mixin update 
RUN colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml 
RUN colcon metadata update

# COPY ./src/perception/ /ws/src/perception/
# COPY ./src/moa/moa_description /ws/src/moa/moa_description
# COPY ./src/moa/moa_msgs /ws/src/moa/moa_msgs

# install ros2 packages
WORKDIR /ws/src/ 
RUN git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git 
RUN source /opt/ros/humble/setup.bash 
RUN rosdep update 
RUN apt-get update 
RUN rosdep install --from-paths /ws/src --ignore-src -r -y --rosdistro=$ROS_DISTRO --os=ubuntu:jammy 
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean

WORKDIR /usr/local/zed 
RUN python3 -m pip install requests 
RUN python3 -m pip install cython numpy opencv-python pyopengl
RUN python3 get_python_api.py

WORKDIR /ws

RUN colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ \
        --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
        ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
        ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'
# DCMAKE_LIBRARY_PATH might cause issues on windows machines
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]