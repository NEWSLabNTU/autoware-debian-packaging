# Run `apt update` to refresh package caches
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

echo 'info: run apt update'
sudo apt update

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    echo 'info: install dependencies'
    ./generate-rosdep-commands.sh | parallel --tty
    ./extra-deps.sh
else
    echo 'info: skip installing dependencies'
fi
