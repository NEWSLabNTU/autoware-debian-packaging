# colcon2deb

Build Debian packages from colcon workspaces using Docker containers.

## Overview

`colcon2deb` is a tool that converts ROS 2 packages in a colcon workspace into Debian packages. It runs the build process inside Docker containers to ensure a clean and reproducible build environment.

## Installation

### From Source

```bash
# Clone the repository
git clone https://github.com/autowarefoundation/colcon2deb.git
cd colcon2deb

# Build and install the Debian package
make deb
sudo dpkg -i colcon2deb_*.deb
```

### Using makedeb

```bash
# Install makedeb if not already installed
# See: https://www.makedeb.org/

# Build the package
makedeb -si
```

## Usage

### 1. Prepare a Docker Builder Image

You need a Docker image with ROS 2 and build dependencies installed. You have two options:

**Option A: Use an existing image**
```yaml
docker:
  image: ros:humble-ros-base
```

**Option B: Build from a Dockerfile**
```yaml
docker:
  dockerfile: ./path/to/Dockerfile
```

Note: Pre-configured Dockerfiles for various architectures (amd64, arm64, jetpack) will be available in a separate repository.

### 2. Prepare Your Colcon Workspace

Example using Autoware:

```bash
# Clone Autoware repository
git clone https://github.com/autowarefoundation/autoware.git
cd autoware

# Import dependencies
mkdir src
vcs import src < autoware.repos

# Your workspace is now at: /path/to/autoware
```

### 3. Create a Configuration File

Create a `config.yaml` file:

```yaml
# config.yaml
version: 1

# Docker configuration
docker:
  # Option 1: Use existing image
  image: ros:humble-ros-base
  
  # Option 2: Build from Dockerfile
  # dockerfile: ./Dockerfile
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

## Package-Specific Debian Configurations

If you need to customize the Debian packaging for specific packages, create a directory structure like:

```
config/
├── package_name_1/
│   └── debian/
│       ├── control      # Package metadata
│       ├── rules        # Build rules
│       └── changelog    # Version history
└── package_name_2/
    └── debian/
        └── ...
```

These custom configurations will override the auto-generated Debian files for those packages.

## Example Configuration

A complete example is provided in `/usr/share/colcon2deb/example/` (or in the `example/` directory if running from source):

- `config.yaml` - Example configuration file
- `config/` - Example package-specific Debian configurations

## Build Process

The build process runs entirely inside Docker containers and includes:

1. **Source Preparation** - Copy workspace sources into build directory
2. **Dependency Installation** - Install ROS dependencies using rosdep
3. **Build** - Compile all packages using colcon
4. **Debian Generation** - Create Debian control files for each package
5. **Package Building** - Build `.deb` files using dpkg-buildpackage

## Requirements

- Docker or Docker CE
- Python 3
- PyYAML

## Development

To run from source without installing:

```bash
./colcon2deb --workspace /path/to/workspace --config config.yaml
```

The tool automatically detects whether it's running from an installed location or from the source directory.

## License

Apache License 2.0

## Support

For issues and contributions, please visit: https://github.com/autowarefoundation/colcon2deb