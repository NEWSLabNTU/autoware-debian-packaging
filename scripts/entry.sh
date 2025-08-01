#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,uid:,gid:
PARSED=$(getopt --options "$OPTIONS" --longoptions "$LONGOPTIONS" --name "$0" -- "$@")

# Check if getopt failed
if [[ $? -ne 0 ]]; then
    exit 1
fi

# Evaluate the parsed options
eval set -- "$PARSED"

# Parse arguments
uid=
gid=

print_usage() {
    echo "Usage: $0 --uid=UID --gid=GID"
}

while true; do
    case "$1" in
	-h|--help)
	    print_usage
	    exit 0
	    ;;
	--uid)
	    uid="$2"
	    shift 2
	    ;;
	--gid)
	    gid="$2"
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

if [ -z "$uid" -o -z "$gid" ]; then
    echo "Invalid options" >&2
    print_usage >&2
    exit 1
fi

# Create a user for the specfied uid/gid and fix file permissions
name=ubuntu
groupadd -g "$gid" "$name"
useradd -m -u "$uid" -g "$gid" "$name"
usermod -aG sudo "$name"
passwd -d "$name"
sudo mkdir /workspace
chown -R "$name:$name" /workspace

# Run the build script
sudo -u ubuntu \
     bash -c "rosdep update && '$script_dir/main.sh' --repo=/mount"
