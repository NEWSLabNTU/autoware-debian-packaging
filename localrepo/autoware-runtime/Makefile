.PHONY: build

current_dir = $(shell pwd)

build:
	rm -rf src pkg
	cp ../../../build_deb/log/deb_pkgs.txt packages.txt
	makedeb -d
