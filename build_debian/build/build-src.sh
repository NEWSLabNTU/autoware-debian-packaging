cd "$colcon_work_dir"
source /opt/ros/humble/setup.bash

if [ "$colcon_build" = y ]; then
    echo 'info: compile packages'
    colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release || {
	echo 'error: colcon build failed' >&2
	return 1
    }
else
    echo 'info: skip compiling packages'
fi
