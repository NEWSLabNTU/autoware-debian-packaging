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

See example configurations in `/usr/share/colcon2deb/example/`:
- `config.yaml` - Basic example using pre-built Docker image
- `config-with-dockerfile.yaml` - Example using custom Dockerfile (requires source repo)
- `config/` - Example package-specific Debian configurations

## Helper Scripts

The build process uses helper scripts installed in `/usr/share/colcon2deb/helper/`