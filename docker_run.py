#!/usr/bin/env python3
"""Docker run script replacement for make run with enhanced functionality."""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def run_command(cmd, check=True):
    """Run a shell command and return the result."""
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=check, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    return result


def build_image_from_dockerfile(dockerfile_path, image_name):
    """Build Docker image from Dockerfile."""
    dockerfile_path = Path(dockerfile_path).resolve()
    if not dockerfile_path.exists():
        print(f"Error: Dockerfile not found at {dockerfile_path}", file=sys.stderr)
        sys.exit(1)

    # Build context is the parent directory of the Dockerfile
    build_context = dockerfile_path.parent

    cmd = [
        "docker",
        "build",
        str(build_context),
        "-f",
        str(dockerfile_path),
        "-t",
        image_name,
    ]

    print(f"Building Docker image '{image_name}' from {dockerfile_path}")
    run_command(cmd)
    return image_name


def main():
    parser = argparse.ArgumentParser(
        description="Docker run script with enhanced functionality for Autoware Debian packaging"
    )

    # Image source - either image name or Dockerfile
    image_group = parser.add_mutually_exclusive_group(required=True)
    image_group.add_argument("--image", "-i", help="Docker image name to use")
    image_group.add_argument(
        "--dockerfile", "-f", help="Path to Dockerfile to build and use"
    )

    # Colcon directory
    parser.add_argument(
        "--colcon-dir",
        "-c",
        required=True,
        help="Path to colcon directory to mount at /mount in container",
    )

    # Optional: custom image name when building from Dockerfile
    parser.add_argument(
        "--image-name",
        default="autoware_rosdebian_builder_custom",
        help="Image name to use when building from Dockerfile (default: autoware_rosdebian_builder_custom)",
    )

    # Parse arguments
    args = parser.parse_args()

    # Determine the image to use
    if args.dockerfile:
        image_name = build_image_from_dockerfile(args.dockerfile, args.image_name)
    else:
        image_name = args.image

    # Verify colcon directory exists
    colcon_dir = Path(args.colcon_dir).resolve()
    if not colcon_dir.exists():
        print(f"Error: Colcon directory not found at {colcon_dir}", file=sys.stderr)
        sys.exit(1)

    # Get current user/group IDs
    uid = os.getuid()
    gid = os.getgid()

    # Get the rosdebian project directory (where this script is located)
    script_dir = Path(__file__).resolve().parent

    # Prepare Docker run command
    docker_cmd = [
        "docker",
        "run",
        "-it",
        "--rm",
        "--net=host",
        "--runtime",
        "nvidia",
        "-e",
        f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "-v",
        "/tmp/.X11-unix/:/tmp/.X11-unix",
        "-v",
        f"{colcon_dir}:/mount",
        "-v",
        f"{script_dir}:/mount/rosdebian",
        image_name,
        "/mount/rosdebian/scripts/entry.sh",
        f"--uid={uid}",
        f"--gid={gid}",
    ]

    # Run the container
    print(f"\nStarting container:")
    print(f"  Colcon directory: {colcon_dir} -> /mount")
    print(f"  Rosdebian directory: {script_dir} -> /mount/rosdebian")
    print(f"  Using image: {image_name}")

    try:
        subprocess.run(docker_cmd)
    except subprocess.CalledProcessError as e:
        print(f"Error running container: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()
