FROM ubuntu:jammy

# DEBIAN_FRONTEND=noninteractive prevents apt from asking for user input
# software-properties-common is needed for apt-add-repository
# sudo is needed for ansible since it escalates from a normal user to root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y && apt-get install software-properties-common sudo -y
RUN apt-add-repository ppa:ansible/ansible -y && apt-get install -y git git-lfs ansible

RUN useradd --create-home --groups sudo --shell /bin/zsh mrover
# Give mrover user sudo access with no password
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER mrover
RUN mkdir -p /home/mrover/ros2_ws/src/mrover
WORKDIR /home/mrover/ros2_ws/src/mrover
# Defines the APT packages that need to be installed
# rosdep is called from Ansible to install them
ADD --chown=mrover:mrover ./package.xml .
# Defines the Python packages that need to be installed
# pip is called from Ansible to install them
ADD --chown=mrover:mrover ./pyproject.toml ./README.md ./LICENSE.md .
ADD --chown=mrover:mrover ./mrover ./mrover
# Copy over all Ansible files
ADD --chown=mrover:mrover ./ansible ./ansible
ADD --chown=mrover:mrover ./ansible.sh .
ADD --chown=mrover:mrover ./pkg ./pkg
RUN ./ansible.sh ci.yml

USER root
RUN apt-get purge ansible -y && apt-get autoremove -y
# Remove apt cache to free up space in the image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER mrover
ENTRYPOINT [ "/bin/bash" ]
