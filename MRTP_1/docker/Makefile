WORKSPACE:=MRTP
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
IMAGE:=ghcr.io/ucmercedrobotics/mrtp

# Automatic Docker platform detection based on host architecture
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_M),x86_64)
    PLATFORM := linux/amd64
else ifeq ($(UNAME_M),arm64)
    PLATFORM := linux/arm64
else ifeq ($(UNAME_M),aarch64)
    PLATFORM := linux/arm64
else
    PLATFORM := linux/amd64
endif

PLATFORM_ARG := --platform $(PLATFORM)

shell:
	docker exec -it $(shell docker ps -aq --filter ancestor=${IMAGE}) bash

build-image:
	docker build -f Dockerfile . -t ${IMAGE} --target base

push:
	docker push ${IMAGE}

vnc:
	docker run -d --rm --net=host \
	--platform=${PLATFORM} \
	--name=novnc \
	${NOVNC}

bash:
	cd .. && \
	docker run -it --rm \
	--net=host \
	--privileged \
	-v $(PWD)/..:/${WORKSPACE}:Z \
	-v $(HOME)/.ssh:/root/.ssh:ro \
	${IMAGE} bash

clean:
	rm -rf build/ install/ log/

husky:
	ros2 launch clearpath_gz simulation.launch.py

turtlebot:
	ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

.PHONY: repo-init shell build-image vnc bash clean husky