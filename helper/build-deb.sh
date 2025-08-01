# Build Debian files for each package in topological order.
cd "$colcon_work_dir"
echo 'info: build Debian packages'

njobs=$(( ( $(nproc) + 3 ) / 4 ))
export DEB_BUILD_OPTIONS="parallel=$(nproc)" # Enable parallel compilation

build_deb() {
    set -Eeuo pipefail

    pkg_name="$1"
    shift
    pkg_dir="$1"
    shift
    
    # Prepare the working directory for the package
    pkg_name_dashed="${pkg_name//_/-}"
    pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
    out_file="$pkg_work_dir/build.out"
    err_file="$pkg_work_dir/build.err"
    # status_file="$pkg_work_dir/status"

    truncate -s 0 "$out_file"
    truncate -s 0 "$err_file"

    # If the Debian package is already built, skip the build.
    deb_path=$(find "$release_dir" -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
    if [ -n "$deb_path" ]; then
	echo "info: skip $pkg_name that its Debian package is already built"
	return
    fi

    echo "info: build Debian package for $pkg_name"
    (
	echo "$pkg_dir"
	echo "$pkg_work_dir"
	cd "$pkg_dir"
	
	# Copy Debian packaging scripts
	rm -rf debian .obj-* ros-$ROS_DISTRO-"${pkg_name_dashed}"_*.deb
	cp -r -t "$pkg_dir" "$pkg_work_dir/debian"

	# Build the package
	fakeroot debian/rules binary

	# Save generated .deb/.ddeb files
	deb_path=$(find .. -name ros-${ROS_DISTRO}-${pkg_name_dashed}_'*'.deb | head -n1)
	if [ -n "$deb_path" ]; then
	    mv -v "$deb_path" "$release_dir"
	fi

	ddeb_path=$(find .. -name ros-${ROS_DISTRO}-${pkg_name_dashed}-dbgsym_'*'.ddeb | head -n1)
	if [ -n "$ddeb_path" ]; then
	    mv -v "$ddeb_path" "$release_dir"
	fi
    )  > "$out_file" 2> "$err_file"
    
    echo "info: build successful for ${pkg_name}" 
    echo "$pkg_name" >> "$successful_pkgs_file" ||
	(
	    echo "error: fail to build Debian package for ${pkg_name}" 
	    echo "$pkg_name" >> "$failed_pkgs_file"
	)
}
export -f build_deb

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	pkg_dir=$(realpath "$pkg_dir")
	sem --id $$ "-j${njobs}" build_deb "$pkg_name" "$pkg_dir"
    done
sem --id $$ --wait
