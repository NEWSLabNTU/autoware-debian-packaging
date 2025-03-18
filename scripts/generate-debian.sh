#!/usr/bin/env bash
set -e

print_usage() {
    echo "Usage: $0 PKG_NAME PKG_DIR OUTPUT_DIR" >&2
    exit 1
}

pkg_name="$1"
shift || print_usage

pkg_dir="$1"
shift || print_usage

output_dir="$1"
shift || print_usage

pkg_name_dashed="${pkg_name//_/-}"
pkg_dir=$(realpath "$pkg_dir")
output_dir=$(realpath "$output_dir")

source /opt/ros/humble/setup.bash

cd "$output_dir"
rm -rf debian .obj-*
bloom-generate rosdebian --ros-distro "$ROS_DISTRO" "$pkg_dir"
