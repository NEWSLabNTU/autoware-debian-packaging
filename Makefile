.PHONY: default prepare build save

IMAGE_NAME = autoware_rosdeiban

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
	docker run \
		-it --rm \
		--net=host \
		--runtime nvidia \
		-e DISPLAY=$$DISPLAY \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v $$PWD/..:/mount \
		$(IMAGE_NAME)

save:
	docker save $(IMAGE_NAME) | zstd -T0 -o $(IMAGE_NAME).tar.zstd
