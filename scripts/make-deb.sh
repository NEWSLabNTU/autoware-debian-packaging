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

work_dir="$1"
shift || print_usage

release_dir="$1"
shift || print_usage

pkg_name_dashed="${pkg_name//_/-}"
pkg_dir=$(realpath "$pkg_dir")
work_dir=$(realpath "$work_dir")
release_dir=$(realpath "$release_dir")

# Go to the source directory
cd "$pkg_dir"

# Copy Debian packaging scripts
rm -rf debian .obj-* ros-$ROS_DISTRO-"${pkg_name_dashed}"_*.deb
cp -r -t "$pkg_dir" "$work_dir/debian"

# Build the package
fakeroot debian/rules binary

# Save generated .deb/.ddeb files
deb_path=$(find .. -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
if [ -n "$deb_path" ]; then
    mv -v "$deb_path" "$release_dir"
fi

ddeb_path=$(find .. -name ros-${ROS_DISTRO}-${pkg_name_dashed}-dbgsym_'*'.ddeb | head -n1)
if [ -n "$ddeb_path" ]; then
    mv -v "$ddeb_path" "$release_dir"
fi
