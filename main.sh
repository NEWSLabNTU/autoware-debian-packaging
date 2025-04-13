#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Function to display usage information
usage() {
    echo "Usage: $0 REPO_DIR [--image DOCKER_IMAGE] [--debian-dir DEBIAN_DIR]"
    echo
    echo "  REPO_DIR                 Path to the repository directory (required)"
    echo "  --debian-dir DEBIAN_DIR  Optional path to a directory of Debian packaging files"
    echo "  --image DOCKER_IMAGE     Optional Docker image to use"
    exit 1
}

# Check if at least one argument (REPO_DIR) is provided
if [ $# -lt 1 ]; then
    echo "Error: REPO_DIR is required."
    usage
fi

# Store the required REPO_DIR argument
repo_dir="$1"
shift # Remove REPO_DIR from the argument list

# Initialize default values
docker_image="ghcr.io/autowarefoundation/autoware:universe-devel-cuda-20250205-amd64"
debian_dir=

# Parse optional arguments
while [ $# -gt 0 ]; do
    case "$1" in
	--image)
	    if [ -z "$2" ]; then
		echo "Error: --image requires a Docker image name."
		usage
	    fi
	    docker_image="$2"
	    shift 2
	    ;;
	--debian-dir)
	    if [ -z "$2" ]; then
		echo "Error: --debian-dir requires a path."
		usage
	    fi
	    debian_dir="$2"
	    shift 2
	    ;;
	*)
	    echo "Error: Unknown argument '$1'."
	    usage
	    ;;
    esac
done

# Validate repo_dir
if [ ! -d "$repo_dir" ]; then
    echo "Error: REPO_DIR '$repo_dir' does not exist or is not a directory."
    exit 1
fi

# Print parsed arguments for confirmation
echo "Repository directory: $repo_dir"
echo "Docker image: $docker_image"
if [ -z "$debian_dir" ]; then
    echo "Debian directory: $debian_dir"
fi

# Build Debian packages in a container
user_id=$(id -u)
group_id=$(id -g)

if [ -n "$debian_dir" ]; then
    debian_dir_volume="$debian_dir:/debian"
else
    debian_dir_volume=
fi

rocker \
    --user \
    --network host \
    --x11 \
    --mode interactive \
    --volume \
    "$repo_dir":/repo \
    "$script_dir":/helper \
    "$debian_dir_volume" \
    -- \
    "$docker_image" \
    /helper/build_debian/main.sh --repo=/repo --debian=/debian

# Create a local repository
tmp_dir="$(mktemp -d)"
cp -r -t "$tmp_dir" "$script_dir"/localrepo/*
"$tmp_dir/build.sh" "$repo_dir/build_deb/dist" .
rm -rf "$tmp_dir"
