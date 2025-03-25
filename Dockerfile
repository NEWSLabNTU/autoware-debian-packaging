FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel AS base

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

# Prepare the setup script
USER root
RUN apt install -y parallel fakeroot debhelper dh-python rsync
COPY rosdebian/setup /workspace/setup
RUN chown -R ubuntu:ubuntu /workspace

# Run setup script
USER ubuntu
WORKDIR /workspace/setup
RUN ./setup.sh

# Remove the default user
USER root
RUN userdel -r ubuntu
WORKDIR /

CMD ["/bin/bash"]
