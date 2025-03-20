.PHONY: build

current_dir = $(shell pwd)

build:
	cd ../../build_deb/dist && \
	tar -cf $(current_dir)/packages.tar *.deb *.ddeb
	makedeb -s
