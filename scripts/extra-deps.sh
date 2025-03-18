#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

awk '\
BEGIN {
  n_apt = 0
  n_pip = 0
}

$1 == "apt" {
  apt[n_apt] = $2
  n_apt += 1
}

$1 == "pip" {
  pip[n_pip] = $2
  n_pip += 1
}

$1 != "apt" && $1 != "pip" {
  printf "Error: Unknown command %s\n", $1 > "/dev/stderr"
  exit 1
}

END {
  if (n_apt > 0) {
    printf "sudo apt install -y"
    for (i = 0; i < n_apt; i += 1) {
      printf " %s", apt[i]
    }
    printf "\n"
  }

  if (n_pip > 0) {
    printf "sudo pip install -y"
    for (i = 0; i < n_pip; i += 1) {
      printf " %s", pip[i]
    }
    printf "\n"
  }
}
' < "$script_dir/extra-deps.txt" | \
    while read line; do
	$line
    done
