.PHONY: default prepare build save

IMAGE_NAME = autoware_rosdebian_builder

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

prepare:
	sudo apt-get install qemu binfmt-support qemu-user-static
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes --credential yes
	sudo systemctl restart docker.service

build:
	docker build .. -f Dockerfile -t $(IMAGE_NAME)

run:
	./docker_run.py --dockerfile Dockerfile --colcon-dir ~/repos/autoware/0.45.1-ws/

save:
	docker save $(IMAGE_NAME) | zstd -T0 -o $(IMAGE_NAME).tar.zstd

pack:
	$(MAKE) -C localrepo build
