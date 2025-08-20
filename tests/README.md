# colcon2deb Tests

This directory contains test scripts and configurations for colcon2deb functionality.

## Directory Structure

```
tests/
├── configs/                      # Test configuration files
│   ├── remote-dockerfile.yaml    # Config with remote Dockerfile URL
│   ├── local-dockerfile.yaml     # Config with local Dockerfile path
│   └── prebuilt-image.yaml       # Config with pre-built Docker image
├── test_remote_dockerfile.py     # Test remote Dockerfile download
├── test_remote_config.py         # Test configuration parsing
└── test_integration.sh           # Integration tests
```

## Running Tests

### Test Remote Dockerfile Download
```bash
python3 tests/test_remote_dockerfile.py
```

### Test Configuration Parsing
```bash
python3 tests/test_remote_config.py
```

### Run All Integration Tests
```bash
./tests/test_integration.sh
```

## Test Configurations

The `configs/` directory contains test configuration files that are independent of the main `examples/` directory. This ensures tests remain stable even as examples evolve.

### remote-dockerfile.yaml
Tests downloading and building from a remote Dockerfile URL.

### local-dockerfile.yaml
Tests building from a local Dockerfile using relative paths.

### prebuilt-image.yaml
Tests using a pre-built Docker image from a registry.

## Directory Independence

All test scripts are designed to work regardless of the current working directory. They use absolute paths based on the script location to find resources.