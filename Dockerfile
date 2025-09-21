FROM arm64v8/ros:humble-ros-base-jammy

# Builds mrover source code and ROS2 in a separate mrover user on the image.

# DEBIAN_FRONTEND=noninteractive prevents apt from asking for user input
# software-properties-common is needed for apt-add-repository
# sudo is needed for ansible since it escalates from a normal user to root
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/depot_tools:${PATH}"

ARG PAT
ARG USERNAME

RUN apt-get update -y && apt-get install software-properties-common sudo -y
RUN apt-add-repository ppa:ansible/ansible -y && apt-get install -y git git-lfs ansible

# Installing dependencies
RUN sudo apt-get install    libxrandr-dev libxinerama-dev libxcursor-dev mesa-common-dev \
                            libx11-xcb-dev libeigen3-dev pkg-config git cmake build-essential \
                            python3-pip python3-setuptools xvfb fluxbox x11vnc novnc websockify \
                            mesa-vulkan-drivers vulkan-tools -y
RUN apt-get update \ 
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install -U colcon-common-extensions

RUN export XDG_RUNTIME_DIR=/tmp/runtime-root

RUN --mount=type=bind,source=.,target=/ros2_ws/src/mrover,rw \
    /ros2_ws/src/mrover/ansible.sh ci.yml && \
    echo "source /ros_entrypoint.sh" >> /root/.zshrc && \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> /root/.zshrc && \
    echo "export ZSH=/root/.oh-my-zsh" >> /root/.zshrc

RUN apt-get purge ansible -y && apt-get autoremove -y
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src/mrover
RUN --mount=type=bind,source=.,target=.,rw \
    ./scripts/build_dawn.sh && \
    ./scripts/build_manifpy.sh

RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> .bashrc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && ./build.sh"

RUN chmod +x entrypoint.sh


