#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"
make build
make run
make pack
cp -t . ./localrepo/autoware-localrepo/autoware-localrepo_*.deb
