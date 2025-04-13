.PHONY: help debian

help:
	@echo 'Usage:'
	@echo '    make help      Show this help message'
	@echo '    make debian    Create a Debian package'
	@echo '    make clean     Clean up generated Debian package and artifacts'

debian:
	git archive --format=tar --prefix=pack-rosdeb/ -o pack-rosdeb.tar HEAD
	makedeb -d

clean:
	rm -rf src pkg *.deb
