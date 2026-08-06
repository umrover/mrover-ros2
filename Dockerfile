FROM ubuntu:noble

# DEBIAN_FRONTEND=noninteractive prevents apt from asking for user input
# software-properties-common is needed for apt-add-repository
# sudo is needed for ansible since it escalates from a normal user to root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y && apt-get install software-properties-common sudo -y
RUN apt-add-repository ppa:ansible/ansible -y && apt-get install -y git git-lfs ansible

RUN useradd --create-home --shell /bin/zsh mrover
# using per user rule over sudo group because noble's default %sudo rule 
# requires a password and beats a NOPASSWD one for the same group
RUN echo 'mrover ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/mrover && chmod 0440 /etc/sudoers.d/mrover

USER mrover
RUN mkdir -p /home/mrover/mrover-ros2
WORKDIR /home/mrover/mrover-ros2
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
ADD --chown=mrover:mrover ./scripts ./scripts
RUN ./ansible.sh ci.yml

USER root
RUN apt-get purge ansible -y && apt-get autoremove -y
# Remove apt cache to free up space in the image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER mrover
ENTRYPOINT [ "/bin/bash" ]
