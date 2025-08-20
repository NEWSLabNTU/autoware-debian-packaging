#!/usr/bin/env python3
"""Test that the remote Dockerfile configuration can be loaded and validated."""

import sys
import yaml
from pathlib import Path

# Get paths relative to this test file
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

# Use test configuration file
CONFIG_PATH = SCRIPT_DIR / "configs" / "remote-dockerfile.yaml"

def test_config():
    """Test loading and validating the remote Dockerfile config."""
    
    print("Testing remote Dockerfile configuration")
    print("="*50)
    relative_path = CONFIG_PATH.relative_to(PROJECT_ROOT)
    print(f"Config file: {relative_path}")
    
    if not CONFIG_PATH.exists():
        print(f"❌ Config file not found: {CONFIG_PATH}")
        print(f"   Current working directory: {Path.cwd()}")
        print(f"   Project root: {PROJECT_ROOT}")
        return False
    
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f)
        
        print("✅ Config file loaded successfully")
        
        # Check Docker configuration
        docker_config = config.get('docker', {})
        if 'dockerfile' in docker_config:
            dockerfile_url = docker_config['dockerfile']
            if dockerfile_url.startswith(('http://', 'https://')):
                print(f"✅ Remote Dockerfile URL found: {dockerfile_url}")
            else:
                print(f"❌ Dockerfile is not a URL: {dockerfile_url}")
                return False
        else:
            print("❌ No dockerfile in config")
            return False
        
        # Check other required fields
        if 'output' in config and 'directory' in config['output']:
            print(f"✅ Output directory configured: {config['output']['directory']}")
        else:
            print("❌ Output directory not configured")
            return False
        
        if 'packages' in config and 'directory' in config['packages']:
            print(f"✅ Packages directory configured: {config['packages']['directory']}")
        else:
            print("❌ Packages directory not configured")
            return False
        
        print("\n" + "="*50)
        print("✅ All checks passed!")
        print("Configuration is valid for remote Dockerfile usage")
        return True
        
    except yaml.YAMLError as e:
        print(f"❌ Error parsing YAML: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False

if __name__ == "__main__":
    success = test_config()
    sys.exit(0 if success else 1)