# colcon2deb

Build Debian packages from colcon workspaces using Docker containers.

## Overview

`colcon2deb` is a tool that converts ROS 2 packages in a colcon workspace into Debian packages. It runs the build process inside Docker containers to ensure a clean and reproducible build environment.

## Architecture

```mermaid
graph TD
    %% User inputs
    A[Colcon Workspace] --> C[colcon2deb]
    B[config.yaml] --> C
    E[Package Configs<br/>\(Optional\)] --> C
    
    %% Process
    C --> F[Docker Image]
    F --> D[Build in Container]
    D --> L[.deb Files]
    
    %% Styling
    style A fill:#e1f5fe,stroke:#01579b,color:#000
    style B fill:#fff3e0,stroke:#e65100,color:#000
    style E fill:#fff3e0,stroke:#e65100,color:#000
    style C fill:#f3e5f5,stroke:#4a148c,color:#000
    style F fill:#e8f5e9,stroke:#1b5e20,color:#000
    style D fill:#e8f5e9,stroke:#1b5e20,color:#000
    style L fill:#ffebee,stroke:#b71c1c,color:#000
```

## Installation

### Download Pre-built Package

Download the latest release from [GitHub Releases](https://github.com/NEWSLabNTU/colcon2deb/releases/tag/v0.1.0):

```bash
# Download the .deb package
wget https://github.com/NEWSLabNTU/colcon2deb/releases/download/v0.1.0/colcon2deb_0.1.0-1_all.deb

# Install the package
sudo apt install ./colcon2deb_0.1.0-1_all.deb
```


## Usage

### 1. Prepare a Docker Builder Image

You need a Docker image with ROS 2 and build dependencies installed.

**Option 1: Use an existing ROS image**
```bash
docker pull ros:humble-ros-base
```

**Option 2: Prepare your own Dockerfile**

Pre-configured Dockerfiles for various architectures (amd64, arm64, jetpack) are available at [autoware-build-images](https://github.com/NEWSLabNTU/autoware-build-images).


### 2. Prepare Your Colcon Workspace

Example using Autoware:

```bash
# Clone Autoware repository
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
git checkout 2025.02

# Import dependencies
mkdir src
vcs import src < autoware.repos

# Your workspace is now at: ~/autoware
```

### 3. Create a Configuration File

Create a `config.yaml` file to specify your build settings. Check the `example/` directory for a complete working example with all available options:

```yaml
# config.yaml
version: 1

# Docker configuration
docker:
  # Option 1: Use existing image
  image: ros:humble-ros-base
  
  # Option 2: Build from Dockerfile
  # dockerfile: ./path/to/Dockerfile
  # image_name: my_builder  # optional, name for built image

# Output configuration
output:
  directory: ./output  # Where to place built .deb files

# Package configurations
packages:
  directory: ./config  # Directory containing package-specific debian/ folders

# Build options (optional)
build:
  ros_distro: humble  # auto-detected if not specified
  parallel_jobs: 8    # defaults to nproc
  skip_tests: true    # defaults to false
```

### 4. Run colcon2deb

```bash
colcon2deb --workspace /path/to/autoware --config config.yaml
```

This will:
1. Build or pull the specified Docker image
2. Mount your workspace into the container
3. Build all ROS packages in the workspace
4. Generate Debian packages for each ROS package
5. Output `.deb` files to the specified output directory

## Customization

colcon2deb provides several ways to customize the build process:

- **Docker images**: Choose pre-built images or use custom Dockerfiles
- **Package configurations**: Override auto-generated Debian files for specific packages
- **Build options**: Control parallelism, testing, and ROS distribution

For detailed customization options, see the [Customization Guide](CUSTOMIZATION.md).

## Requirements

- Docker or Docker CE
- Python 3 with PyYAML
- Ubuntu 22.04 (recommended)

## For Developers

If you want to contribute to colcon2deb or build it from source, please see the [Development Guide](DEVELOPMENT.md).

## Support

For issues and contributions, please visit: https://github.com/NEWSLabNTU/colcon2deb

## License

Apache License 2.0
