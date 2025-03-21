.PHONY: build

current_dir = $(shell pwd)

build:
	rm -rf packages
	mkdir packages

	cp -t packages ../../../build_deb/dist/*.deb
	cp -t packages ../ros-humble-autoware-full/ros-humble-autoware-full_*.deb
	cd packages && tar -cf ../packages.tar *.deb

	rm -rf src pkg
	makedeb -s
