# ==============================================================================
# V-LiDAR Autonomous Navigation & Perception Environment
# Target: Intel NUC 14 Pro (Intel Core Ultra 7 155H, Intel Arc iGPU, NPU)
# ROS 2 Humble + Intel OpenVINO + PyTorch (CPU/iGPU) + stella-vslam + Nav2
# ==============================================================================

FROM ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# 1. 시스템 기본 패키지 및 인텔 GPU(Arc) / OpenVINO / VSLAM / Nav2 의존성 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    pkg-config \
    python3-pip \
    python3-colcon-common-extensions \
    # Intel iGPU (Arc Graphics) OpenCL & Level Zero Compute Runtime
    intel-opencl-icd \
    intel-level-zero-gpu \
    level-zero \
    # VSLAM & Pangolin dependencies
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libglew-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    libeigen3-dev \
    ros-${ROS_DISTRO}-libg2o \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-tools \
    v4l-utils \
    udev \
    net-tools \
    iputils-ping \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 2. Pangolin 빌드 및 설치 (stella-vslam 호환 버전)
ARG PANGOLIN_COMMIT=ad8b5f83222291c51b4800d5a5873b0e90a0cf81
WORKDIR /tmp
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout ${PANGOLIN_COMMIT} && \
    sed -i -e "193,198d" ./src/utils/file_utils.cpp && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
          -DBUILD_PANGOLIN_FFMPEG=OFF \
          -DBUILD_PANGOLIN_LIBDC1394=OFF \
          -DBUILD_PANGOLIN_LIBJPEG=OFF \
          -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
          -DBUILD_PANGOLIN_LIBPNG=OFF \
          -DBUILD_PANGOLIN_LIBREALSENSE=OFF \
          -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
          -DBUILD_PANGOLIN_LIBTIFF=OFF \
          -DBUILD_PANGOLIN_LIBUVC=OFF \
          -DBUILD_PANGOLIN_LZ4=OFF \
          -DBUILD_PANGOLIN_OPENNI=OFF \
          -DBUILD_PANGOLIN_OPENNI2=OFF \
          -DBUILD_PANGOLIN_PLEORA=OFF \
          -DBUILD_PANGOLIN_PYTHON=OFF \
          -DBUILD_PANGOLIN_TELICAM=OFF \
          -DBUILD_PANGOLIN_TOON=OFF \
          -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
          -DBUILD_PANGOLIN_V4L=OFF \
          -DBUILD_PANGOLIN_VIDEO=OFF \
          -DBUILD_PANGOLIN_ZSTD=OFF \
          -DBUILD_PYPANGOLIN_MODULE=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf Pangolin

ENV Pangolin_DIR=/usr/local/lib/cmake/Pangolin

# 3. Python AI 라이브러리 설치
# Intel NUC 14 Pro 최적화: 경량 PyTorch(CPU) + Intel OpenVINO 런타임 + Ultralytics
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir \
    openvino \
    openvino-dev \
    ultralytics \
    transforms3d \
    pyserial \
    supabase \
    numpy \
    opencv-python

# 4. 워크스페이스 디렉토리 및 엔트리포인트
WORKDIR /root/ros2_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
