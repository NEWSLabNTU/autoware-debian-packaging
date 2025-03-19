#!/usr/bin/env bash

# Here is the environment when this setup script is executed.
#
# - USER=ubuntu
# - GROUP=ubuntu
# - CWD=/workspace/setup
# - UID=1000
# - GID=1000

set -e

# Run Ansible script
./setup-dev-env.sh -y --no-cuda-drivers
rosdep update
