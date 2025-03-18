FROM ubuntu:22.04 AS base

# Update system packages
RUN apt update -y
RUN apt upgrade -y

# Configure the locale and timezone
RUN apt install -y locales
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen
RUN locale-gen
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN ln -snf /usr/share/zoneinfo/Asia/Taipei /etc/localtime && echo Asia/Taipei > /etc/timezone

# Enable 'universe' repository
RUN apt install -y software-properties-common
RUN apt install -y apt-transport-https
RUN add-apt-repository universe

# Install APT packages
RUN apt install -y python3-venv
RUN apt install -y python3-pip
RUN apt install -y sudo
RUN apt install -y openssh-client
RUN apt install -y openssh-server

# Create the 'ubuntu' user
RUN useradd -m -u 1000 ubuntu
RUN usermod -aG sudo ubuntu
RUN passwd --delete ubuntu

# Switch to the 'ubuntu' user
RUN mkdir /mount
RUN mkdir /workspace
WORKDIR /workspace
RUN chown ubuntu:ubuntu /workspace
USER ubuntu

# Set default process to bash
CMD ["/bin/bash"]


FROM base AS builder

COPY setup-dev-env.sh /workspace/setup-dev-env.sh
COPY ansible /workspace/ansible
COPY amd64.env /workspace/amd64.env
COPY arm64.env /workspace/arm64.env
COPY ansible-galaxy-requirements.yaml /workspace/ansible-galaxy-requirements.yaml
RUN sudo chown -R ubuntu:ubuntu /workspace
RUN ./setup-dev-env.sh -y --no-cuda-drivers
RUN rosdep update

RUN sudo apt install -y parallel fakeroot debhelper dh-python
COPY rosdebian/scripts /workspace/scripts

USER root
RUN userdel -r ubuntu

CMD ["/bin/bash"]
