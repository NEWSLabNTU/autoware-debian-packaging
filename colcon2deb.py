#!/usr/bin/env python3
"""Docker run script replacement for make run with enhanced functionality."""

import argparse
import os
import subprocess
import sys
import yaml
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


def load_config(config_path):
    """Load configuration from YAML file."""
    config_path = Path(config_path).resolve()
    if not config_path.exists():
        print(f"Error: Config file not found at {config_path}", file=sys.stderr)
        sys.exit(1)

    with open(config_path, "r") as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"Error parsing config file: {e}", file=sys.stderr)
            sys.exit(1)

    return config


def main():
    parser = argparse.ArgumentParser(
        description="Build Debian packages from colcon workspace"
    )

    # Workspace directory
    parser.add_argument(
        "--workspace",
        required=True,
        help="Path to colcon workspace directory",
    )

    # Config file
    parser.add_argument(
        "--config",
        required=True,
        help="Path to configuration YAML file",
    )

    # Parse arguments
    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config)

    # Validate config version
    if config.get("version") != 1:
        print(
            f"Error: Unsupported config version: {config.get('version')}",
            file=sys.stderr,
        )
        sys.exit(1)

    # Get docker configuration
    docker_config = config.get("docker", {})
    if "image" in docker_config and "dockerfile" in docker_config:
        print(
            "Error: Cannot specify both 'image' and 'dockerfile' in config",
            file=sys.stderr,
        )
        sys.exit(1)

    if "image" not in docker_config and "dockerfile" not in docker_config:
        print(
            "Error: Must specify either 'image' or 'dockerfile' in config",
            file=sys.stderr,
        )
        sys.exit(1)

    # Determine the image to use
    if "dockerfile" in docker_config:
        dockerfile_path = Path(docker_config["dockerfile"])
        if not dockerfile_path.is_absolute():
            # Relative to config file
            dockerfile_path = Path(args.config).parent / dockerfile_path
        # Ensure absolute path
        dockerfile_path = dockerfile_path.resolve()
        image_name = build_image_from_dockerfile(
            dockerfile_path, docker_config.get("image_name", "colcon2deb_builder")
        )
    else:
        image_name = docker_config["image"]

    # Verify workspace directory exists
    workspace_dir = Path(args.workspace).resolve()
    if not workspace_dir.exists():
        print(
            f"Error: Workspace directory not found at {workspace_dir}", file=sys.stderr
        )
        sys.exit(1)

    # Get current user/group IDs
    uid = os.getuid()
    gid = os.getgid()

    # Get the script directory (where this script is located)
    script_dir = Path(__file__).resolve().parent

    # Helper directory must be in the same directory as this script
    helper_dir = script_dir / "helper"

    if not helper_dir.exists():
        print("Error: Helper scripts directory not found", file=sys.stderr)
        print(f"Expected at: {helper_dir}", file=sys.stderr)
        print("Helper directory must be in the same directory as colcon2deb.py", file=sys.stderr)
        sys.exit(1)

    # Get packages configuration directory
    packages_config = config.get("packages", {})
    if "directory" not in packages_config:
        print("Error: 'packages.directory' not specified in config", file=sys.stderr)
        sys.exit(1)

    packages_dir = Path(packages_config["directory"])
    if not packages_dir.is_absolute():
        # Relative to config file
        packages_dir = Path(args.config).parent / packages_dir

    # Ensure absolute path for Docker
    packages_dir = packages_dir.resolve()

    if not packages_dir.exists():
        print(
            f"Error: Packages config directory not found at {packages_dir}",
            file=sys.stderr,
        )
        sys.exit(1)

    # Get output directory configuration - this will be our main working directory
    output_config = config.get("output", {})
    if "directory" not in output_config:
        print("Error: 'output.directory' not specified in config", file=sys.stderr)
        sys.exit(1)

    output_dir = Path(output_config["directory"])
    if not output_dir.is_absolute():
        # Relative to config file
        output_dir = Path(args.config).parent / output_dir

    # Ensure absolute path for Docker
    output_dir = output_dir.resolve()

    # Create output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)

    # The build_deb directory will be created inside the output directory
    # Users will find packages in output_dir/dist/

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
        f"{workspace_dir}:/workspace",
        "-v",
        f"{packages_dir}:/config",
        "-v",
        f"{helper_dir}:/helper",
        "-v",
        f"{output_dir}:/output",
        image_name,
        "/helper/entry.sh",
        f"--uid={uid}",
        f"--gid={gid}",
        f"--output=/output",
    ]

    # Run the container
    print(f"\nStarting container:")
    print(f"  Workspace directory: {workspace_dir} -> /workspace")
    print(f"  Packages config directory: {packages_dir} -> /config")
    print(f"  Output directory: {output_dir} -> /output")
    print(f"  Helper directory: {helper_dir} -> /helper")
    print(f"  Using image: {image_name}")
    print(f"\n  Build artifacts will be in: {output_dir}/")
    print(f"  Final .deb packages will be in: {output_dir}/dist/")

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
