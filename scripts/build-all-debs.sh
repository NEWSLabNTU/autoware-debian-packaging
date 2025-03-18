#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

# Locate utility scripts
script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
make_deb_file="$script_dir/make-deb.sh"
rosdep_gen_file="$script_dir/generate-rosdep-commands.sh"

# Locate input/output files
mount_dir=/mount
top_work_dir="$mount_dir/build_deb"
success_file="$top_work_dir/success"
fail_file="$top_work_dir/fail"
rosdep_script_file="$top_work_dir/rosdep.sh"
rosdep_yaml_file=$(realpath "$top_work_dir")/rosdep.yaml
rosdep_list_file="/etc/ros/rosdep/sources.list.d/99-autoware.list"

# Prepare the working directory
sudo mkdir -p "$top_work_dir"
sudo chmod 777 "$top_work_dir"
cd "$top_work_dir"

# Copy source files
echo 'Copying source files...'
cp -a -t "$top_work_dir" "$mount_dir/src"

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
echo 'Install rosdep dependencies'
"$rosdep_gen_file"  > "$rosdep_script_file"
chmod +x "$rosdep_script_file"
"$rosdep_script_file" || {
    exit 1
}

# List packages from the repository in the rosdep file and perform
# `rosdep update`
echo 'Generate rosdep list'
colcon list | cut -f1 | \
    sort | \
    while read pkg_name; do
	pkg_name_dashed="${pkg_name//_/-}"
	key="ros-${ROS_DISTRO}-${pkg_name_dashed}"
	echo "\
${pkg_name}:
  ubuntu: [${key}]"
    done > "$rosdep_yaml_file"

sudo sh -c "echo 'yaml file://${rosdep_yaml_file}' > '$rosdep_list_file'"
rosdep update

# Build Debian files for each package in topological order.
echo 'Start building Debian packages'
colcon list --topological-order | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_name_dashed="${pkg_name//_/-}"
	pkg_work_dir="$(realpath $top_work_dir/$pkg_name)"
	out_file="$pkg_work_dir/out"
	err_file="$pkg_work_dir/err"
	status_file="$pkg_work_dir/status"
	mkdir -p "$pkg_work_dir"

	# If the Debian package is already built, skip the build.
	deb_path=$(find "$pkg_work_dir" -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
	if [ -n "$deb_path" ]; then
	    echo "SKIP $pkg_name"
	    continue
	fi
	echo "BUILD $pkg_name"

	# Build and install the Debian package.
	(
	    "$make_deb_file" "$pkg_name" "$pkg_dir" "$pkg_work_dir" && \
		sudo dpkg -i $pkg_work_dir/ros-$ROS_DISTRO-"${pkg_name_dashed}"_*.deb && \
		sudo apt-get install -f
	)  > "$out_file" 2> "$err_file"

	# Save the status and show the results.
	status=$?
	echo $status > "$status_file"

	if [ $status -eq 0 ]; then
	    echo "SUCCESS $pkg_name"
	    echo "$pkg_name $pkg_dir" >> "$success_file"
	else
	    echo "FAIL $pkg_name"
	    echo "$pkg_name $pkg_dir" >> "$fail_file"
	    exit 1
	fi
    done
