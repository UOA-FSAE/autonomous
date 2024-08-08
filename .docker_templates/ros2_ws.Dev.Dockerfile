FROM osrf/ros:humble-desktop
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws

# setup sources.list and keys
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list 
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#install general packages
RUN apt-get update 
RUN apt-get install --no-install-recommends -y  \
    ros-humble-foxglove-bridge \
    tzdata \
    dirmngr \
    gnupg2 \
    git \
    ros-humble-ros-base \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool 
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# setup colcon mixin and metadata
RUN rosdep init 
RUN rosdep update --rosdistro $ROS_DISTRO 
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 
RUN colcon mixin update 
RUN colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml 
RUN colcon metadata update


RUN source /opt/ros/humble/setup.bash 
RUN rosdep update --rosdistro $ROS_DISTRO 
RUN apt-get update 
RUN rosdep install --from-paths src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy 
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean

RUN source /opt/ros/humble/setup.bash 
RUN colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ --base-paths src \
        --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
        ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
        ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD [ "bash" ]