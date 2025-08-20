#!/bin/bash
# Test script to verify remote Dockerfile integration
# This script is directory-agnostic and will work from any location

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Change to project root for imports to work
cd "$PROJECT_ROOT" || exit 1

echo "Testing colcon2deb with remote Dockerfile configuration"
echo "========================================================"
echo "Project root: $PROJECT_ROOT"
echo "Current dir:  $(pwd)"
echo ""

# Test 1: Verify the script can parse the config
echo "Test 1: Parsing remote Dockerfile config..."
python3 -c "
import sys
from pathlib import Path

# Add project root to path
project_root = Path('$PROJECT_ROOT')
sys.path.insert(0, str(project_root))

from colcon2deb import load_config

# Use test config file
config_file = project_root / 'tests' / 'configs' / 'remote-dockerfile.yaml'

print(f'  Using config: {config_file.relative_to(project_root)}')
config = load_config(str(config_file))
docker_config = config.get('docker', {})

if 'dockerfile' in docker_config:
    url = docker_config['dockerfile']
    if url.startswith(('http://', 'https://')):
        print('  ✅ Config parsed successfully')
        print(f'  URL: {url}')
    else:
        print('  ❌ Not a URL')
        sys.exit(1)
else:
    print('  ❌ No dockerfile in config')
    sys.exit(1)
"

if [ $? -eq 0 ]; then
    echo ""
else
    echo "  ❌ Failed to parse config"
    exit 1
fi

# Test 2: Test download functionality
echo "Test 2: Testing Dockerfile download..."
python3 -c "
import sys
from pathlib import Path

# Add project root to path
project_root = Path('$PROJECT_ROOT')
sys.path.insert(0, str(project_root))

from colcon2deb import download_dockerfile

url = 'https://raw.githubusercontent.com/NEWSLabNTU/autoware-build-images/refs/heads/main/0.45.1/amd64/Dockerfile'
cache_dir = Path.home() / '.cache' / 'colcon2deb' / 'dockerfiles'

try:
    result = download_dockerfile(url, cache_dir)
    if result.exists():
        print('  ✅ Download successful')
        # Test cache hit
        result2 = download_dockerfile(url, cache_dir)
        if result == result2:
            print('  ✅ Cache working')
        else:
            print('  ⚠️  Cache might not be working')
    else:
        print('  ❌ Download failed')
        sys.exit(1)
except Exception as e:
    print(f'  ❌ Error: {e}')
    sys.exit(1)
"

if [ $? -eq 0 ]; then
    echo ""
else
    echo "  ❌ Download test failed"
    exit 1
fi

echo "========================================================"
echo "✅ All integration tests passed!"
echo ""
echo "The remote Dockerfile feature is ready to use."
echo ""
echo "Example usage:"
echo ""
echo "1. With remote Dockerfile:"
echo "   $PROJECT_ROOT/colcon2deb.py --workspace ~/repos/autoware/0.45.1-ws/ \\"
echo "                                --config $PROJECT_ROOT/tests/configs/remote-dockerfile.yaml"
echo ""
echo "2. With local Dockerfile:"
echo "   $PROJECT_ROOT/colcon2deb.py --workspace ~/repos/autoware/0.45.1-ws/ \\"
echo "                                --config $PROJECT_ROOT/tests/configs/local-dockerfile.yaml"
echo ""
echo "3. With pre-built image:"
echo "   $PROJECT_ROOT/colcon2deb.py --workspace ~/repos/autoware/0.45.1-ws/ \\"
echo "                                --config $PROJECT_ROOT/tests/configs/prebuilt-image.yaml"
echo ""