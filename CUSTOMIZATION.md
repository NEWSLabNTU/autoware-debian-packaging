# Customization Guide

This guide explains how to customize various aspects of the colcon2deb build process.

## Configuration Directory Structure

A typical configuration setup looks like this:

```
my-build/
├── config.yaml                 # Main configuration file
├── config/                     # Package-specific Debian configurations
│   ├── package_name_1/
│   │   └── debian/
│   │       ├── control        # Package metadata
│   │       ├── rules          # Build rules
│   │       ├── changelog      # Version history
│   │       ├── compat         # Debhelper compatibility
│   │       └── copyright      # License information
│   └── package_name_2/
│       └── debian/
│           └── ...
└── Dockerfile                  # Optional: Custom Docker image

# The colcon workspace is separate:
/path/to/workspace/
└── src/
    ├── package_name_1/
    ├── package_name_2/
    └── ...
```

## Docker Image Selection

### Using Pre-built Images

The easiest way is to use existing ROS 2 Docker images:

```yaml
docker:
  image: ros:humble-ros-base
```

Available ROS images:
- `ros:humble-ros-base` - Minimal ROS 2 Humble
- `ros:humble-desktop` - ROS 2 Humble with GUI tools
- `ros:humble-desktop-full` - ROS 2 Humble with all packages

### Using Custom Dockerfiles

For more control, use a custom Dockerfile:

```yaml
docker:
  dockerfile: ./my-dockerfile
  image_name: my_custom_builder  # Optional: name for the built image
```

Example Dockerfile structure:
```dockerfile
FROM ubuntu:22.04

# Install ROS 2
# ... (ROS installation steps)

# Install your specific dependencies
RUN apt-get update && apt-get install -y \
    your-dependency-1 \
    your-dependency-2

# Install development tools
RUN apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions
```

### Pre-configured Dockerfiles

Ready-to-use Dockerfiles for various platforms are available at [autoware-build-images](https://github.com/NEWSLabNTU/autoware-build-images):

- **amd64**: Standard x86_64 architecture
- **arm64**: ARM 64-bit architecture
- **jetpack**: NVIDIA Jetson platforms

## Package-Specific Debian Configurations

### When to Use Custom Configurations

You might need custom Debian configurations when:
- A package requires specific dependencies not detected by bloom
- You need custom build rules or install scripts
- The auto-generated control file needs modifications
- You want to split a package into multiple Debian packages

### Directory Structure

Create custom configurations in your packages directory:

```
config/
├── my_package/
│   └── debian/
│       ├── control      # Package metadata
│       ├── rules        # Build rules
│       ├── changelog    # Version history
│       ├── compat       # Debhelper compatibility level
│       └── copyright    # License information
└── another_package/
    └── debian/
        └── ...
```

### Essential Files

#### control
Defines package metadata:
```
Source: my-package
Section: misc
Priority: optional
Maintainer: Your Name <your.email@example.com>
Build-Depends: debhelper (>= 9), 
               ros-humble-rclcpp,
               ros-humble-std-msgs
Standards-Version: 3.9.8

Package: ros-humble-my-package
Architecture: any
Depends: ${shlibs:Depends}, 
         ${misc:Depends},
         ros-humble-rclcpp,
         ros-humble-std-msgs
Description: My ROS 2 package
 Detailed description of what this package does.
```

#### rules
Build instructions (Makefile format):
```makefile
#!/usr/bin/make -f

%:
	dh $@

override_dh_auto_configure:
	# Custom configuration steps

override_dh_auto_build:
	# Custom build steps

override_dh_auto_install:
	# Custom installation steps
```

#### changelog
Version history:
```
my-package (1.0.0-1) unstable; urgency=low

  * Initial release

 -- Your Name <your.email@example.com>  Mon, 01 Jan 2024 00:00:00 +0000
```

### Example: Autoware Package

The `example/config/` directory contains real examples from Autoware packages. For instance:

```
example/config/
├── autoware_tensorrt_yolox/
│   └── debian/
│       ├── control
│       ├── rules
│       └── changelog
```

## Build Options

### Configuration File Options

```yaml
build:
  ros_distro: humble      # ROS distribution (auto-detected if not set)
  parallel_jobs: 8        # Number of parallel jobs (defaults to nproc)
  skip_tests: true        # Skip running tests during build
```

### Environment Variables

You can also control the build through environment variables:

```bash
export ROS_DISTRO=humble
export MAKEFLAGS="-j8"
```

## Advanced Customization

### Multiple Configuration Files

Create different configurations for different scenarios:

```bash
# Development build
colcon2deb --workspace /path/to/ws --config config-dev.yaml

# Release build
colcon2deb --workspace /path/to/ws --config config-release.yaml
```

### Build Subsets of Packages

To build only specific packages, create a custom packages directory with configurations only for those packages.

### Custom Helper Scripts

Advanced users can modify the helper scripts, but this requires rebuilding the colcon2deb package. See the [Development Guide](DEVELOPMENT.md) for details.

## Troubleshooting Custom Configurations

### Common Issues

1. **Missing dependencies**: Check the build logs in the container
2. **Incorrect paths**: Ensure all paths in config.yaml are correct
3. **Bloom generation fails**: Your package might need a custom debian/ directory

### Debug Mode

Run the container interactively to debug:

```bash
docker run -it --rm \
  -v /path/to/workspace:/mount \
  -v /path/to/config:/config \
  -v /path/to/helper:/helper \
  your-image:tag \
  /bin/bash
```

Then manually run the helper scripts to identify issues.

## Best Practices

1. **Start simple**: Use auto-generation first, then customize only what's needed
2. **Version control**: Keep your custom configurations in version control
3. **Document changes**: Add comments explaining why customizations were needed
4. **Test thoroughly**: Verify packages install and run correctly
5. **Keep it minimal**: Only override what's necessary to reduce maintenance