#!/usr/bin/env bash
set -e

# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,skip-rosdep-install,skip-copy-src,skip-gen-rosdep-list,skip-colcon-build,repo:
PARSED=$(getopt --options "$OPTIONS" --longoptions "$LONGOPTIONS" --name "$0" -- "$@")

# Check if getopt failed
if [[ $? -ne 0 ]]; then
    exit 1
fi

# Evaluate the parsed options
eval set -- "$PARSED"

# Parse arguments
export rosdep_install=y
export gen_rosdep_list=y
export copy_src=y
export colcon_build=y
export repo_dir=

print_usage() {
    echo "Usage: $0 [OPTION]... --repo=REPO_DIR"
    echo "  -h,--help                   show this help message"
    echo "  --skip-rosdep-install       do not run `rosdep install`"
    echo "  --skip-copy-src             do not copy source files to the build cache"
    echo "  --skip-gen-rosdep-list      do not modify the system rosdep list"
    echo "  --skip-colcon-build         do not run `colcon build`"
}

while true; do
    case "$1" in
	-h|--help)
	    print_usage
	    exit 0
	    ;;
	--skip-rosdep-install)
	    rosdep_install=n
	    shift 1
	    ;;
	--skip-copy-src)
	    copy_src=n
	    shift 1
	    ;;
	--skip-gen-rosdep-list)
	    gen_rosdep_list=n
	    shift 1
	    ;;
	--skip-colcon-build)
	    colcon_build=n
	    shift 1
	    ;;
	--repo)
	    repo_dir="$2"
	    shift 2
	    ;;
	--)
	    shift
	    break
	    ;;
	*)
	    echo "error: invalid option: ${1}" >&2
	    print_usage >&2
	    exit 1
	    ;;
    esac
done

if [ -z "$repo_dir" ]; then
    echo "--repo REPO_DIR must be provided" >&2
    exit
fi

# Locate utility scripts
export script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
export repo_dir=$(realpath "$repo_dir")

# Set working directory to the parent of this script.
cd "$script_dir"

export top_work_dir="$repo_dir/build_deb"
export colcon_work_dir="$top_work_dir/sources"
export config_dir="/config"
export release_dir="$top_work_dir/dist"
export pkg_build_dir="$top_work_dir/build"

export log_dir="$top_work_dir/log"
export deb_pkgs_file="$log_dir/deb_pkgs.txt"
export successful_pkgs_file="$log_dir/successful_pkgs.txt"
export failed_pkgs_file="$log_dir/failed_pkgs.txt"

# export generate_debian_script="$script_dir/generate-debian.sh"
export rosdep_gen_script="$script_dir/generate-rosdep-commands.sh"
export extra_deps_script="$script_dir/extra-deps.sh"
export make_deb_script="$script_dir/make-deb.sh"

make_pkg_work_dir() {
    pkg_name="$1"
    shift || return 1
    echo "$pkg_build_dir/$pkg_name"
}
export -f make_pkg_work_dir

make_pkg_config_dir() {
    pkg_name="$1"
    shift || return 1
    echo "$config_dir/$pkg_name"
}
export -f make_pkg_config_dir

# Prepare the working directory
./prepare.sh

# Copy source files
./copy-src.sh

# Install dependencies
./install-deps.sh

# Compile the whole repository
./build-src.sh
source "$colcon_work_dir/install/setup.bash"

# Create a rosdep list file
./create-rosdep-list.sh

# Create a Debian package list file
./create-package-list.sh

# Copy or generate Debian control/rules files
./generate-debian-dir.sh

# Build Debian packages
./build-deb.sh
