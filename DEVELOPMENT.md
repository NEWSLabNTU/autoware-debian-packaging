# Development Guide

This guide is for developers and contributors working on colcon2deb.

## Building from Source

### Quick Build and Install

```bash
# Clone the repository
git clone https://github.com/NEWSLabNTU/colcon2deb.git
cd colcon2deb

# Build and install the Debian package
make deb
sudo dpkg -i colcon2deb_*.deb
```

## Development Setup

### Prerequisites

- Python 3.8+
- Docker
- makedeb (for building packages)
- Git

### Clone and Setup

```bash
git clone https://github.com/NEWSLabNTU/colcon2deb.git
cd colcon2deb

# Create a virtual environment (optional but recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install pyyaml
```

## Running from Source

When developing, you can run colcon2deb directly from source:

```bash
./colcon2deb --workspace /path/to/workspace --config config.yaml
```

The script automatically detects whether it's running from an installed location or source directory.

## Building Packages

### Using makedeb

If you have makedeb installed, you can build the Debian package directly:

```bash
makedeb -si
```

This will:
- Build the package (`-s` syncs dependencies)
- Install the package (`-i` installs after building)

### Using Make

The Makefile provides several useful targets:

```bash
# Create source tarball
make tarball

# Build Debian package (creates tarball first)
make deb

# Clean build artifacts
make clean
```

## Project Structure

```
colcon2deb/
├── colcon2deb          # Main executable (Python script)
├── helper/             # Bash scripts that run inside Docker
│   ├── main.sh         # Main orchestrator script
│   ├── prepare.sh      # Prepare build environment
│   ├── copy-src.sh     # Copy source files
│   ├── install-deps.sh # Install dependencies
│   ├── build-src.sh    # Build with colcon
│   └── ...             # Other build scripts
├── example/            # Example configuration
│   ├── config.yaml     # Example config file
│   └── config/         # Example package configs
├── docker/             # Dockerfiles (separate repo)
├── PKGBUILD           # makedeb package definition
└── Makefile           # Build automation
```

## Code Architecture

### Main Script (colcon2deb)

The main script handles:
1. Command-line argument parsing
2. Configuration file loading and validation
3. Docker image management (build/pull)
4. Container execution with proper mounts
5. Helper script detection (installed vs development)

### Helper Scripts

Helper scripts run inside the Docker container and handle:
- Source preparation
- Dependency installation
- Package building
- Debian file generation
- Package creation

### Configuration

Configuration is handled through YAML files with schema validation:
- `version`: Config format version
- `docker`: Docker image/Dockerfile settings
- `output`: Output directory for .deb files
- `packages`: Directory with custom Debian configs
- `build`: Build options (ROS distro, parallelism, etc.)

## Testing

### Manual Testing

1. Create a test workspace with a simple ROS package
2. Create a test configuration
3. Run colcon2deb and verify output

### Integration Testing

Test with real workspaces:
- Small ROS 2 workspace
- Autoware (large workspace)
- Custom packages with specific dependencies

## Contributing

### Code Style

- Python: Follow PEP 8
- Bash: Use shellcheck for linting
- YAML: 2-space indentation

### Pull Request Process

1. Fork the repository
2. Create a feature branch
3. Make changes and test
4. Submit PR with clear description
5. Ensure CI passes

### Adding Features

When adding new features:
1. Update configuration schema if needed
2. Add documentation to README.md
3. Update example configuration
4. Test with various workspaces

## Release Process

1. Update version in:
   - `PKGBUILD` (pkgver)
   - `Makefile` (VERSION)
   - Any other version references

2. Create release commit:
   ```bash
   git commit -m "Release v0.1.0"
   git tag v0.1.0
   ```

3. Build release package:
   ```bash
   make clean
   make deb
   ```

4. Create GitHub release with:
   - Release notes (use RELEASE.md as template)
   - Upload .deb package
   - Upload source tarball

## Debugging

### Debug Docker Issues

```bash
# Run container interactively
docker run -it --rm \
  -v /path/to/workspace:/mount \
  -v /path/to/config:/config \
  -v /path/to/helper:/helper \
  ros:humble-ros-base \
  /bin/bash

# Then manually run helper scripts
/helper/main.sh --repo=/mount
```

### Debug Build Issues

Check log files in the build directory:
- `build_deb/log/` - General logs
- `build_deb/build/*/gen_deb.out` - Package-specific logs
- `build_deb/build/*/gen_deb.err` - Package-specific errors

### Common Issues

1. **Permission errors**: Ensure Docker can access mounted directories
2. **Missing dependencies**: Check rosdep installation logs
3. **Build failures**: Check individual package build logs
4. **Debian generation errors**: Verify custom debian/ directories

## Future Improvements

- [ ] Add progress reporting
- [ ] Support for incremental builds
- [ ] Parallel package building
- [ ] Better error handling and reporting
- [ ] Configuration validation tool
- [ ] Support for cross-compilation