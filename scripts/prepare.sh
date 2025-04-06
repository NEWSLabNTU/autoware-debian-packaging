echo 'info: prepare working directories'

truncate -s 0 "$deb_pkgs_file" 2>/dev/null
truncate -s 0 "$successful_pkgs_file" 2>/dev/null
truncate -s 0 "$failed_pkgs_file" 2>/dev/null

mkdir -p "$top_work_dir"
mkdir -p "$colcon_work_dir"
mkdir -p "$release_dir"
mkdir -p "$log_dir"
mkdir -p "$pkg_build_dir"

# Prevent colcon from erroneously scan this folder
touch "$top_work_dir/COLCON_IGNORE"

clean_work_dir() {
    set -e
    work_dir=$1
    shift
    mkdir -p "$work_dir" 
    rm -f "$work_dir"/*.out "$work_dir"/*.err
}
export -f clean_work_dir

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_dir=$(realpath "$pkg_dir")
	pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
	sem "-j$(nproc)" clean_work_dir "$pkg_work_dir"
    done
