#!/usr/bin/env bash

# Here is the environment when this setup script is executed.
#
# - USER=ubuntu
# - GROUP=ubuntu
# - CWD=/workspace/setup
# - UID=1000
# - GID=1000

set -e

# Remove /usr/local CMake binaries
sudo rm -f /usr/local/bin/cmake /usr/local/bin/cpack /usr/local/bin/ctest

sudo apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
sudo tee /etc/apt/sources.list.d/nvidia-l4t-apt-source.list > /dev/null <<EOF
deb https://repo.download.nvidia.com/jetson/common r36.3 main
deb https://repo.download.nvidia.com/jetson/t234 r36.3 main
deb https://repo.download.nvidia.com/jetson/ffmpeg r36.3 main
EOF

sudo mkdir -p /opt/nvidia/l4t-packages
sudo touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall

sudo apt update
sudo apt install -o DPkg::Options::="--force-confold" -y nvidia-l4t-dla-compiler

# Run Ansible script
./setup-dev-env.sh -y --no-cuda-drivers --no-nvidia
rosdep update
