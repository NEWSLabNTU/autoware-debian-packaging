.PHONY: default prepare build save tarball clean deb

IMAGE_NAME = autoware_rosdebian_builder
VERSION = 0.1.0

default:
	@echo 'Usage:'
	@echo
	@echo 'make prepare'
	@echo '    Install dependent pakcages and configure the'
	@echo '    environment to build the container image.'
	@echo
	@echo 'make build'
	@echo '    Build the container image.'
	@echo
	@echo 'make run'
	@echo '    Enter the container shell.'
	@echo
	@echo 'make save'
	@echo '    Save a Docker image file.'
	@echo
	@echo 'make tarball'
	@echo '    Create a source tarball for packaging.'
	@echo
	@echo 'make deb'
	@echo '    Create a Debian package using makedeb.'

prepare:
	sudo apt-get install qemu binfmt-support qemu-user-static
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes --credential yes
	sudo systemctl restart docker.service

build:
	docker build .. -f Dockerfile -t $(IMAGE_NAME)

run:
	./colcon2deb --workspace ~/repos/autoware/0.45.1-ws/ --config example/config.yaml

save:
	docker save $(IMAGE_NAME) | zstd -T0 -o $(IMAGE_NAME).tar.zstd

pack:
	$(MAKE) -C localrepo build

tarball:
	@echo "Creating source tarball for colcon2deb v$(VERSION)..."
	@mkdir -p dist
	@tar --transform 's,^,colcon2deb-$(VERSION)/,' \
		--exclude='example/docker' \
		--exclude='*.pyc' \
		--exclude='__pycache__' \
		-czf dist/colcon2deb-$(VERSION).tar.gz \
		colcon2deb \
		helper/ \
		example/ \
		PKGBUILD \
		README.md
	@echo "Tarball created: dist/colcon2deb-$(VERSION).tar.gz"

clean:
	@rm -rf dist/
	@rm -rf pkg/
	@rm -rf src/
	@rm -f *.deb
	@rm -f *.tar.gz
	@echo "Cleaned up build artifacts"

deb: tarball
	@echo "Building Debian package with makedeb..."
	@cp dist/colcon2deb-$(VERSION).tar.gz .
	makedeb -s
	@echo "Debian package created successfully!"
