# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This project builds Debian packages for Autoware (autonomous driving platform) in isolated Docker containers. It implements a **two-program architecture**:

1. **Host-side CLI** (`src/colcon_debian_packager/`) - Python orchestrator
2. **Container-side scripts** (`scripts/`) - Bash build system

The output is a local APT repository (`autoware-localrepo_*.deb`) containing all Autoware packages.

## Key Commands

### Build Everything
```bash
./start.sh  # One-shot build process
```

### Development Commands
```bash
# Prepare environment
make prepare

# Build Docker container
make build

# Enter container shell
make run

# Build repository packages
make pack

# Run tests
source .venv/bin/activate
pytest
pytest --cov  # with coverage

# Code quality
black src/
flake8 src/
mypy src/
```

### Python CLI (colcon-deb)
```bash
colcon-deb init-config              # Initialize config
colcon-deb build /path/to/autoware  # Build packages
colcon-deb clean                    # Clean artifacts
colcon-deb validate                 # Validate config
```

## Architecture

### Directory Structure
- `src/colcon_debian_packager/` - Python CLI implementation
  - `cli/` - Command interface
  - `components/` - Debian/ROS builders
  - `services/` - Docker and workspace management
  - `models/` - Data models
- `scripts/` - Container-side build scripts
  - `main.sh` - Primary entry point
  - Individual scripts for each build phase
- `config/` - Pre-configured Debian templates for problematic packages
- `setup/` - Ansible playbooks for environment setup
- `tests/` - Test structure (unit/integration)

### Build Process Flow
1. `prepare.sh` - Initialize working directory
2. `copy-src.sh` - Copy Autoware source
3. `install-deps.sh` - Install ROS dependencies
4. `build-src.sh` - Compile with colcon
5. `create-rosdep-list.sh` - Generate dependency list
6. `generate-debian-dir.sh` - Create Debian metadata
7. `build-deb.sh` - Build .deb packages

### Key Design Principles
- **No ROS on host** - Only Python and Docker required
- **Read-only source mounts** - Prevents accidental modifications
- **Isolated builds** - Everything runs in Docker containers
- **Clean interfaces** - Communication via volumes, env vars, exit codes

## Testing

Tests use pytest with a virtual environment at `.venv/`:

```bash
# Run specific test
pytest tests/unit/cli/test_build.py

# Run all tests
pytest

# Test a specific component
pytest tests/unit/services/test_docker_service.py
```

## Important Files

- `src/colcon_debian_packager/cli/build.py` - Main build command implementation
- `scripts/main.sh` - Container-side build orchestrator
- `config/*/debian/` - Package-specific Debian configurations
- `doc/ARCHITECTURE-OVERVIEW.md` - Detailed architecture documentation

## Common Development Tasks

### Adding New Package Configuration
1. Create directory in `config/<package_name>/`
2. Add `debian/control`, `debian/rules`, `debian/changelog`
3. Test with individual package build

### Debugging Build Issues
1. Use `make run` to enter container
2. Run individual scripts manually in `/workdir/`
3. Check logs in build directory

### Modifying Build Process
- Host-side changes: Edit Python code in `src/colcon_debian_packager/`
- Container-side changes: Edit scripts in `scripts/`
- Always maintain separation between host and container responsibilities