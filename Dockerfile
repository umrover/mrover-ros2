FROM ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive

# Add LLVM 18 apt repository
RUN apt-get update -y && apt-get install -y curl ca-certificates git && \
    curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key \
      -o /usr/share/keyrings/llvm-archive-keyring.asc && \
    echo "deb [signed-by=/usr/share/keyrings/llvm-archive-keyring.asc] \
      http://apt.llvm.org/noble/ llvm-toolchain-noble-18 main" \
      > /etc/apt/sources.list.d/llvm.list

# Add ROS 2 Jazzy apt repository
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros.list

RUN apt-get update -y && apt-get install -y \
    cmake \
    ccache \
    ninja-build \
    curl \
    rsync \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    clang-18 \
    clang-tidy-18 \
    clang-format-18 \
    lld-18 \
    gcc-13 \
    g++-13 \
    libbullet-dev \
    libglfw3-dev \
    libx11-xcb-dev \
    libnl-3-dev \
    libnl-route-3-dev \
    libtbb-dev \
    libassimp-dev \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    ros-jazzy-ros-base \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-rtcm-msgs \
    ros-jazzy-magic-enum \
    ros-jazzy-dynamixel-sdk \
    libboost-dev \
    qt6-multimedia-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages "git+https://github.com/artivis/manif.git"

RUN rosdep init && rosdep update

RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 130 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 130 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-18 180 && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-18 180 && \
    update-alternatives --install /usr/bin/lld lld /usr/bin/lld-18 180 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-18 180

ENTRYPOINT ["/bin/bash"]
