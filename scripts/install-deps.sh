# Run `apt update` to refresh package caches
echo 'info: run apt update'
sudo apt update

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    echo 'info: install dependencies'
    ./generate-rosdep-commands.sh
    ./extra-deps.sh
else
    echo 'info: skip installing dependencies'
fi
