FROM nvcr.io/nvidia/l4t-base:35.4.1
LABEL Name=autonomous Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ws

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libopenblas-dev \
        libopenmpi-dev \
        openmpi-bin \
        openmpi-common \
        gfortran \
        libomp-dev  \
        nvidia-cuda-dev \
        nvidia-cudnn8-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

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

ENV ROS_DISTRO humble

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
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
    
COPY . .

RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO && apt-get update && \
    cd /ws && \
    rosdep install --from-paths src -y --ignore-src -i src/perception --rosdistro=$ROS_DISTRO --os=ubuntu:jammy && \ 
    rm -rf /var/lib/apt/lists/* 

ENV PYTORCH_URL=https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl PYTORCH_WHL=torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl 

RUN cd /opt && \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
    pip3 install --verbose ${PYTORCH_WHL}

RUN python3 -c 'import torch; print(f"PyTorch version: {torch.__version__}"); print(f"CUDA available:  {torch.cuda.is_available()}"); print(f"cuDNN version:   {torch.backends.cudnn.version()}"); print(torch.__config__.show());'

RUN source /opt/ros/humble/setup.bash && \
    colcon build --parallel-workers $(nproc) --symlink-install \
        --event-handlers console_direct+ --base-paths src \
        --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
        ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
        ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

CMD [ "bash" ]
