# colcon2deb

Build Debian packages from colcon workspaces using Docker containers.

## Usage

```bash
colcon2deb --workspace /path/to/workspace --config /path/to/config.yaml
```

## Configuration

Create a config.yaml file with the following structure:

```yaml
version: 1

docker:
  # Option 1: Use existing image
  image: ros:humble-ros-base
  
  # Option 2: Build from Dockerfile
  # dockerfile: ./Dockerfile

output:
  directory: ./output

packages:
  directory: ./config  # Directory with package-specific debian/ folders

build:
  ros_distro: humble
  parallel_jobs: 8
  skip_tests: true
```

## Example

See example configuration and package configs in `/usr/share/colcon2deb/example/`

## Helper Scripts

The build process uses helper scripts installed in `/usr/share/colcon2deb/helper/`