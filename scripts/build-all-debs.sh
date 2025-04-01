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
    echo "Usage: $0 [OPTION]... --repo=REPO_DIR"
    echo "  -h,--help                   show this help message"
    echo "  --skip-rosdep-install       do not run `rosdep install`"
    echo "  --skip-copy-src             do not copy source files to the build cache"
    echo "  --skip-gen-rosdep-list      do not modify the system rosdep list"
    echo "  --skip-colcon-build         do not run `colcon build`"
    echo "  --skip-install-extra-deps   do not install extra dependencies"
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
	    echo "error: invalid option: ${1}" >&2
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
colcon_work_dir="$top_work_dir/sources"
config_dir="$repo_dir/rosdebian/config"
release_dir="$top_work_dir/dist"
pkg_build_dir="$top_work_dir/build"
log_dir="$top_work_dir/log"
deb_pkgs_file="$log_dir/deb_pkgs.txt"
successful_pkgs_file="$log_dir/successful_pkgs.txt"
failed_pkgs_file="$log_dir/failed_pkgs.txt"
generate_debian_script="$script_dir/generate-debian.sh"
rosdep_gen_script="$script_dir/generate-rosdep-commands.sh"
extra_deps_script="$script_dir/extra-deps.sh"
make_deb_script="$script_dir/make-deb.sh"

truncate -s 0 "$deb_pkgs_file"
truncate -s 0 "$successful_pkgs_file"
truncate -s 0 "$failed_pkgs_file"


make_pkg_work_dir() {
    pkg_name="$1"
    shift || return 1
    echo "$pkg_build_dir/$pkg_name"
}

make_pkg_config_dir() {
    pkg_name="$1"
    shift || return 1
    echo "$config_dir/$pkg_name"
}

# Prepare the working directory
echo 'info: prepare working directories'
cd "$repo_dir"

mkdir -p "$top_work_dir"
mkdir -p "$colcon_work_dir"
mkdir -p "$release_dir"
mkdir -p "$log_dir"
mkdir -p "$pkg_build_dir"

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_dir=$(realpath "$pkg_dir")
	pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
	pkg_config_dir="$(make_pkg_config_dir $pkg_name)"

	cat<<EOF
mkdir -p '$pkg_work_dir' && \
rm -f '$pkg_work_dir'/*.out '$pkg_work_dir'/*.err
EOF
    done | parallel --lb

# Copy source files
cd "$colcon_work_dir"

if [ "$copy_src" = y ]; then
    echo 'info: copy source files'
    rsync -avP --delete "$repo_dir/src/" "$colcon_work_dir/src" || {
	echo "error: fail to copy source files" >&2
	return 1
    }
else
    echo 'info: skip copying source files'
fi

# Run `apt update` to refresh package caches
sudo apt update || {
    echo 'error: `apt update` failed' >&2
    exit 1
}

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    echo 'info: install rosdep dependencies'
    "$rosdep_gen_script" | bash - || {
	echo "error: fail to install rosdep dependencies" >&2
	exit 1
    }
else
    echo 'info: skip installing rosdep dependencies'
fi

# Install extra dependencies
if [ "$install_extra_deps" = y ]; then
    "$extra_deps_script" || {
	echo "error: fail to install extra dependencies" >&2
	exit 1
    }
else
    echo 'info: skip installing extra dependencies'
fi

# Compile the whole repository
cd "$colcon_work_dir"
source /opt/ros/humble/setup.bash || {
    echo 'error: fail to source /opt/ros/humble/setup.bash. is ROS installed?' >&2
    return 1
}

if [ "$colcon_build" = y ]; then
    echo 'info: compile packages'
    colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release || {
	echo 'error: colcon build failed' >&2
	return 1
    }
else
    echo 'info: skip compiling packages'
fi

source install/setup.bash

# Generate a rosdep file to include packages and perform `rosdep
# update`. The step is necessary for later bloom-generate.
cd "$colcon_work_dir"

rosdep_yaml_file="$top_work_dir/rosdep.yaml"
rosdep_list_file="/etc/ros/rosdep/sources.list.d/99-autoware.list"

if [ "$gen_rosdep_list" = y ]; then
    echo 'info: generate rosdep list'
    colcon list --base-paths src | cut -f1 | \
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
else
    echo 'info: skip generating rosdep list'
fi

# Build Debian files for each package in topological order.
cd "$colcon_work_dir"
echo 'info: generate Debian packaging scripts'

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_name_dashed="${pkg_name//_/-}"
	pkg_dir=$(realpath "$pkg_dir")
	pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
	pkg_config_dir="$(make_pkg_config_dir $pkg_name)"
	out_file="$pkg_work_dir/gen_deb.out"
	err_file="$pkg_work_dir/gen_deb.err"
	src_debian_dir="$config_dir/$pkg_name/debian"
	dst_debian_dir="$pkg_work_dir/debian"

	mkdir -p "$pkg_work_dir"
	mkdir -p "$pkg_config_dir"
	truncate -s 0 "$out_file"
	truncate -s 0 "$err_file"

	# Skip if the `debian` already exists
	if [ -d "$src_debian_dir" ]; then
	    cat <<EOF
echo 'info: copy provided debian directory for $pkg_name'; \
( \
  rsync -av --delete '$src_debian_dir/' '$dst_debian_dir/' \
) >> '$out_file' 2>> '$err_file' || \
echo "error: fail to copy debian directory for ${pkg_name}" >&2
EOF
	else
	    cat <<EOF
echo 'info: run bloom-generate for $pkg_name'; \
( \
  cd '$pkg_config_dir' && \
  bloom-generate rosdebian --ros-distro '$ROS_DISTRO' '$pkg_dir' && \
  rsync -av --delete '$src_debian_dir/' '$dst_debian_dir/' \
) >> '$out_file' 2>> '$err_file' || \
( \
  echo "error: fail to generate Debain files for ${pkg_name}" >&2 && \
  echo '$pkg_name' >> '$failed_pkgs_file' \
)
EOF
	fi
    done | parallel --lb

#
colcon list --base-paths src | cut -f1 | \
    while read -r pkg_name; do
	echo ros-"${ROS_DISTRO}"-"${pkg_name//_/-}"
    done > "$deb_pkgs_file"

# Build Debian files for each package in topological order.
cd "$colcon_work_dir"
echo 'info: build Debian packages'

njobs=$(( ( $(nproc) + 3 ) / 4 ))
export DEB_BUILD_OPTIONS="parallel=$(nproc)" # Enable parallel compilation

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_name_dashed="${pkg_name//_/-}"
	pkg_dir=$(realpath "$pkg_dir")
	pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
	out_file="$pkg_work_dir/build.out"
	err_file="$pkg_work_dir/build.err"
	# status_file="$pkg_work_dir/status"

	truncate -s 0 "$out_file"
	truncate -s 0 "$err_file"

	# If the Debian package is already built, skip the build.
	deb_path=$(find "$release_dir" -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
	if [ -n "$deb_path" ]; then
	    echo "echo 'info: skip $pkg_name that its Debian package is already built'"
	    continue
	fi

	cat <<EOF
echo 'info: build Debian package for $pkg_name' && \
'$make_deb_script' '$pkg_name' '$pkg_dir' '$pkg_work_dir' '$release_dir' > '$out_file' 2> '$err_file' && \
echo "info: build successful for ${pkg_name}" && \
echo '$pkg_name' >> '$successful_pkgs_file' || \
( \
  echo "error: fail to build Debian package for ${pkg_name}" && \
  echo '$pkg_name' >> '$failed_pkgs_file' \
)
EOF
    done | parallel "-j${njobs}" --lb
