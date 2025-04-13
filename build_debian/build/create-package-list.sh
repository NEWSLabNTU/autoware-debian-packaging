echo 'info: generate Debian package list'

cd "$colcon_work_dir"

colcon info --base-paths src | awk '\
$0 ~ /^  name: / {
  name = $2
  gsub("_", "-", name)
  name = "ros-humble-" name
}

$0 ~ /^    version: / {
  version=$2
  printf "%s=%s-0jammy\n", name, version
}
'  > "$deb_pkgs_file"
