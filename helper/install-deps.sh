# Run `apt update` to refresh package caches
echo 'info: run apt update'
sudo apt update

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    echo 'info: install dependencies'
    # Generate the install script and execute it
    install_script=$(mktemp /tmp/install_deps_XXXXXX.sh)
    ./generate-rosdep-commands.sh > "$install_script"

    # Show what will be installed
    echo "info: generated install script:" >&2
    cat "$install_script" >&2

    # Execute the installation
    bash "$install_script" || {
        echo "error: failed to install dependencies" >&2
        rm -f "$install_script"
        exit 1
    }
    rm -f "$install_script"
else
    echo 'info: skip installing dependencies'
fi
