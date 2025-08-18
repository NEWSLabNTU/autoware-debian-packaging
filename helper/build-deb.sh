# Build Debian files for each package in topological order.
cd "$colcon_work_dir"
echo 'info: build Debian packages'

njobs=$(( ( $(nproc) + 3 ) / 4 ))
export DEB_BUILD_OPTIONS="parallel=$(nproc)" # Enable parallel compilation

build_deb() {
    # Don't use set -e here as we need to handle errors ourselves
    set -uo pipefail

    pkg_name="$1"
    shift
    pkg_dir="$1"
    shift

    # Prepare the working directory for the package
    pkg_name_dashed="${pkg_name//_/-}"
    pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
    out_file="$pkg_work_dir/build.out"
    err_file="$pkg_work_dir/build.err"

    # Set up trap to record failure if script exits unexpectedly
    trap "echo '$pkg_name' >> '$failed_pkgs_file'; echo 'error: unexpected exit for $pkg_name'" ERR EXIT

    truncate -s 0 "$out_file"
    truncate -s 0 "$err_file"

    # If the Debian package is already built, skip the build.
    # Check in the check_dir (either output_dir if specified, or release_dir)
    # Note: Don't quote the pattern in find command to allow glob expansion
    deb_path=$(find "$check_dir" -name "ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb" 2>/dev/null | head -n1)
    if [ -n "$deb_path" ]; then
	echo "info: skip $pkg_name that its Debian package is already built"
	echo "$pkg_name" >> "$skipped_pkgs_file"
	trap - ERR EXIT  # Clear trap before returning
	return
    fi

    echo "info: build Debian package for $pkg_name"

    # Run the build in a subshell and capture the exit status
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
	deb_path=$(find .. -name "ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb" | head -n1)
	if [ -n "$deb_path" ]; then
	    mv -v "$deb_path" "$release_dir"
	fi

	ddeb_path=$(find .. -name "ros-${ROS_DISTRO}-${pkg_name_dashed}-dbgsym_*.ddeb" | head -n1)
	if [ -n "$ddeb_path" ]; then
	    mv -v "$ddeb_path" "$release_dir"
	fi
    ) > "$out_file" 2> "$err_file"

    # Store the exit status
    build_exit_status=$?

    # Check if the .deb file was actually created and moved
    pkg_deb_in_release=$(find "$release_dir" -name "ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb" 2>/dev/null | head -n1)

    # Debug output for problematic packages
    if [ "$pkg_name" = "awapi_awiv_adapter" ] || [ "$pkg_name" = "autoware_perception_online_evaluator" ]; then
        echo "DEBUG: Package $pkg_name build details:"
        echo "  Build exit status: $build_exit_status"
        echo "  Expected pattern: ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb"
        echo "  Looking in: $release_dir"
        echo "  Found .deb file: ${pkg_deb_in_release:-NONE}"

        # Check if any .deb files were created at all
        any_deb=$(find "$pkg_work_dir/.." -name "*.deb" 2>/dev/null | head -n3)
        if [ -n "$any_deb" ]; then
            echo "  Any .deb files in parent directory:"
            echo "$any_deb" | sed 's/^/    /'
        else
            echo "  No .deb files found in parent directory"
        fi

        # Check the actual build output
        echo "  Last 5 lines of build output:"
        tail -5 "$out_file" 2>/dev/null | sed 's/^/    /' || echo "    No output file"

        echo "  Last 5 lines of build errors:"
        tail -5 "$err_file" 2>/dev/null | sed 's/^/    /' || echo "    No error file"
    fi

    # Check the exit status and verify the package was created
    if [ $build_exit_status -eq 0 ] && [ -n "$pkg_deb_in_release" ]; then
	echo "info: build successful for ${pkg_name}"
	echo "$pkg_name" >> "$successful_pkgs_file"
	trap - ERR EXIT  # Clear trap on success
    elif [ $build_exit_status -eq 0 ] && [ -z "$pkg_deb_in_release" ]; then
	echo "warning: build command succeeded but no .deb file found for ${pkg_name}"
	echo "$pkg_name" >> "$failed_pkgs_file"
	trap - ERR EXIT  # Clear trap on failure
    else
	echo "error: fail to build Debian package for ${pkg_name}"
	echo "$pkg_name" >> "$failed_pkgs_file"
	trap - ERR EXIT  # Clear trap on failure
    fi
}
export -f build_deb

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	pkg_dir=$(realpath "$pkg_dir")
	sem --id $$ "-j${njobs}" build_deb "$pkg_name" "$pkg_dir"
    done
sem --id $$ --wait
