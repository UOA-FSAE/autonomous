FROM stereolabs/zed:4.0-devel-cuda12.1-ubuntu22.04
LABEL Name=zed_sdk Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY ../ /

# setup sources.list and keys
#RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list && \
#    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup timezone & install packages
#RUN apt-get update && apt-get install -q -y --no-install-recommends \
 #   tzdata \
 #   dirmngr \
 #   gnupg2 \
 #   git \
 #   ros-humble-ros-core=0.10.0-1* \
 #   build-essential \
 #   python3-colcon-common-extensions \
 #   python3-colcon-mixin \
 #   python3-rosdep \
 #   python3-vcstool \
 #   ros-humble-foxglove-bridge

# setup environment
#ENV LANG C.UTF-8
#ENV LC_ALL C.UTF-8

#ENV ROS_DISTRO humble

# setup colcon mixin and metadata
#RUN rosdep init && \
#    rosdep update --rosdistro $ROS_DISTRO && \
#    colcon mixin add default \
#      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
#    colcon mixin update && \
#    colcon metadata add default \
#      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
#    colcon metadata update

# # install ros2 packages
# RUN mkdir /ws/src/ && cd "$_" && \
#     git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
#     cd .. && \
#     source /opt/ros/humble/setup.bash && \ 
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build --parallel-workers $(nproc) --symlink-install \
#     --event-handlers console_direct+ --base-paths src \
#     --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
#     ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
#     ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' && \
    # rm -rf /var/lib/apt/lists/* && \
    # echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    # echo "source /ws/install/setup.bash" >> ~/.bashrc

# ENTRYPOINT ["../ros_entrypoint.sh"]
CMD ["bash"]
