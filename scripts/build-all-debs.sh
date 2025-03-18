#!/usr/bin/env bash

# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,skip-rosdep-install,skip-copy-src,skip-gen-rosdep-list,skip-colcon-build,repo:,skip-install-extra-deps
PARSED=$(getopt --options "$OPTIONS" --longoptions "$LONGOPTIONS" --name "$0" -- "$@")

# Check if getopt failed
if [[ $? -ne 0 ]]; then
    exit 1
fi

# Evaluate the parsed options
eval set -- "$PARSED"

# Parse arguments
rosdep_install=y
gen_rosdep_list=y
copy_src=y
colcon_build=y
repo_dir=
install_extra_deps=y

print_usage() {
    echo "Usage: $0 [--skip-rosdep-install] --repo=REPO_DIR"
}

while true; do
    case "$1" in
	-h|--help)
	    print_usage
	    exit 0
	    ;;
	--skip-rosdep-install)
	    rosdep_install=n
	    shift 1
	    ;;
	--skip-copy-src)
	    copy_src=n
	    shift 1
	    ;;
	--skip-gen-rosdep-list)
	    gen_rosdep_list=n
	    shift 1
	    ;;
	--skip-colcon-build)
	    colcon_build=n
	    shift 1
	    ;;
	--skip-install-extra-deps)
	    install_extra_deps=n
	    shift 1
	    ;;
	--repo)
	    repo_dir="$2"
	    shift 2
	    ;;
	--)
	    shift
	    break
	    ;;
	*)
	    echo "Invalid option: $1" >&2
	    print_usage >&2
	    exit 1
	    ;;
    esac
done

if [ -z "$repo_dir" ]; then
    echo "--repo REPO_DIR must be provided" >&2
    exit
fi

# Locate utility scripts
script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
repo_dir=$(realpath "$repo_dir")
top_work_dir="$repo_dir/build_deb"
colcon_work_dir="$top_work_dir/_sources"
release_dir="$repo_dir/release_deb"
generate_debian_script="$script_dir/generate-debian.sh"
rosdep_gen_script="$script_dir/generate-rosdep-commands.sh"
extra_deps_script="$script_dir/extra-deps.sh"
make_deb_script="$script_dir/make-deb.sh"

# Locate input/output files
# rosdep_script_file="$colcon_work_dir/rosdep.sh"
# rosdep_yaml_file="$top_work_dir/rosdep.yaml"
# rosdep_list_file="/etc/ros/rosdep/sources.list.d/99-autoware.list"

# Prepare the working directory
sudo mkdir -p "$top_work_dir"
sudo chmod 777 "$top_work_dir"

sudo mkdir -p "$colcon_work_dir"
sudo chmod 777 "$colcon_work_dir"

sudo mkdir -p "$release_dir"
sudo chmod 777 "$release_dir"


# Copy source files
cd "$colcon_work_dir"

if [ "$copy_src" = y ]; then
    echo 'Copying source files...'
    rsync -avP "$repo_dir/src/" "$colcon_work_dir/src"
else
    echo 'Skip copying source files'
fi

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    echo 'Install rosdep dependencies'
    "$rosdep_gen_script" | bash - || {
	echo "ERROR: Fail to install rosdep dependencies"
	exit 1
    }
    # "$rosdep_gen_script"  > "$rosdep_script_file"
    # chmod +x "$rosdep_script_file"
    # "$rosdep_script_file" || {
    # 	echo "ERROR: Fail to install rosdep dependencies"
    # 	exit 1
    # }
else
    echo 'Skip installing rosdep dependencies'
fi

# Install extra dependencies
if [ "$install_extra_deps" = y ]; then
    "$extra_deps_script" || {
	echo "ERROR: Fail to install extra dependencies"
	exit 1
    }
else
    echo 'Skip installing extra dependencies'
fi

# Compile the whole repository
source /opt/ros/humble/setup.bash

if [ "$colcon_build" = y ]; then
    echo 'Compile packages'
    colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release || return 1
else
    echo 'Skip compiling packages'
fi

# Build Debian files for each package in topological order.
echo 'Generate Debian packaging scripts'

make_pkg_work_dir() {
    base_dir="$1"
    shift || return 1
    pkg_name="$1"
    shift || return 1
    echo "$base_dir/$pkg_name"
}

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_name_dashed="${pkg_name//_/-}"
	pkg_dir=$(realpath "$pkg_dir")
	pkg_work_dir="$(make_pkg_work_dir $top_work_dir $pkg_name)"
	out_file="$pkg_work_dir/gen_deb.out"
	err_file="$pkg_work_dir/gen_deb.err"
	debian_dir="$pkg_work_dir/debian"
	
	mkdir -p "$pkg_work_dir"
	truncate -s 0 "$out_file"
	truncate -s 0 "$err_file"
	
	# Skip if the `debian` already exists
	if [ -d "$debian_dir" ]; then
	    echo "echo 'SKIP BLOOM_GENERATE $pkg_name'"
	    continue
	fi
	echo "echo 'BLOOM_GENERATE $pkg_name'"

	# Create `debian` directory.
	cat <<EOF
"$generate_debian_script" '$pkg_name' '$pkg_dir' '$pkg_work_dir' >> '$out_file' 2>> '$err_file' || \
echo 'FAIL $pkg_name' >&2
EOF
    done | parallel --lb

# # List packages from the repository in the rosdep file and perform
# # `rosdep update`
# if [ "$gen_rosdep_list" = y ]; then
#     echo 'Generate rosdep list'
#     colcon list | cut -f1 | \
# 	sort | \
# 	while read pkg_name; do
# 	    pkg_name_dashed="${pkg_name//_/-}"
# 	    key="ros-${ROS_DISTRO}-${pkg_name_dashed}"
# 	    echo "\
# ${pkg_name}:
#   ubuntu: [${key}]"
# 	done > "$rosdep_yaml_file"

#     sudo sh -c "echo 'yaml file://${rosdep_yaml_file}' > '$rosdep_list_file'"
#     rosdep update
# else
#     echo 'Skip generating rosdep list'
# fi

# Build Debian files for each package in topological order.
echo 'Build Debian packages'

source install/setup.bash
njobs=$(( ($(nproc) + 1) / 2 ))

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_name_dashed="${pkg_name//_/-}"
	pkg_dir=$(realpath "$pkg_dir")
	pkg_work_dir="$(make_pkg_work_dir $top_work_dir $pkg_name)"
	out_file="$pkg_work_dir/build.out"
	err_file="$pkg_work_dir/build.err"
	# status_file="$pkg_work_dir/status"
	
	mkdir -p "$pkg_work_dir"
	truncate -s 0 "$out_file"
	truncate -s 0 "$err_file"

	# If the Debian package is already built, skip the build.
	deb_path=$(find "$release_dir" -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
	if [ -n "$deb_path" ]; then
	    echo "echo 'SKIP BUILD $pkg_name'"
	    continue
	fi

	cat <<EOF
echo 'BUILD $pkg_name' && \
'$make_deb_script' '$pkg_name' '$pkg_dir' '$pkg_work_dir' '$release_dir' > '$out_file' 2> '$err_file' && \
echo 'SUCCESS $pkg_name' || \
echo 'FAIL $pkg_name'
EOF
    done | parallel "-j${njobs}" --lb
