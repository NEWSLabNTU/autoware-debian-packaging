#!/usr/bin/env python3
import sys
import os

# Add the installation directory to Python path
sys.path.insert(0, '/usr/share/colcon2deb')

# Import and run the main script
if __name__ == "__main__":
    # Change to the installation directory so helper/ is found
    os.chdir('/usr/share/colcon2deb')
    
    # Execute the main script
    with open('/usr/share/colcon2deb/colcon2deb.py', 'r') as f:
        exec(f.read())