FROM nvcr.io/nvidia/l4t-base:35.4.1
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws

# setup timezone
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup environment
ENV ROS_DISTRO humble

RUN apt update 
RUN apt install locales  
RUN locale-gen en_US en_US.UTF-8 
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt update 
RUN apt install -y gnupg wget software-properties-common 
RUN add-apt-repository universe

RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | \
    apt-key add - 
RUN echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | \
    tee -a "/etc/apt/sources.list"

RUN apt update 
RUN apt install curl -y 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN apt-get update 
RUN apt-get install -y --no-install-recommends \
        libopenblas-dev \
        libopenmpi-dev \
        openmpi-bin \
        openmpi-common \
        gfortran \
        libomp-dev  \
        nvidia-cuda-dev \
        nvidia-cudnn8-dev 
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean

RUN apt update 
RUN apt install --no-install-recommends -y \
    ros-humble-ros-base \
    ros-dev-tools \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \ 
    python3-pip \
    ros-humble-foxglove-bridge 
RUN rm -rf /var/lib/apt/lists/* 
RUN apt-get clean 
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 

RUN rosdep init 
RUN rosdep update --rosdistro $ROS_DISTRO 
RUN apt-get update 
WORKDIR /ws 
RUN rosdep install --from-paths /ws/src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy 
RUN rm -rf /var/lib/apt/lists/* 

ENV PYTORCH_URL=https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl PYTORCH_WHL=torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl 

RUN cd /opt 
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} 
RUN pip3 install --verbose ${PYTORCH_WHL}

RUN python3 -c 'import torch; print(f"PyTorch version: {torch.__version__}"); print(f"CUDA available:  {torch.cuda.is_available()}"); print(f"cuDNN version:   {torch.backends.cudnn.version()}"); print(torch.__config__.show());'

RUN source /opt/ros/humble/setup.bash 
RUN colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ --base-paths src 

CMD [ "bash" ]