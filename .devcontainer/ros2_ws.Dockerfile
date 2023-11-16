FROM nvcr.io/nvidia/l4t-base:35.4.1
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws
COPY . .

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

RUN apt update && apt install --no-install-recommends -y \
    ros-humble-ros-base \
    ros-dev-tools \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \ 
    python3-pip \
    ros-humble-foxglove-bridge && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 

RUN rosdep init && rosdep update && apt-get update && \
    cd /ws && \
    rosdep install --from-paths src -y --ignore-src --rosdistro=humble --os=ubuntu:jammy && \ 
    rm -rf /var/lib/apt/lists/* 

RUN source /opt/ros/humble/setup.bash && colcon build && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc 

CMD [ "bash" ]
