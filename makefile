.SUFFIXES:
.SILENT:


ifeq ($(OS),Windows_NT)
PREREQS = docker docker-compose code
GPU := $(shell wmic path win32_VideoController get name | findstr "NVIDIA")
	ifneq ($(strip $(GPU)),)
GPU_ID := $(shell nvidia-smi --query-gpu=index --format=csv,noheader | findstr "^[0-9]")
	endif
K := $(foreach exec,$(PREREQS),\
	$(if $(shell where $(exec)),some string,$(error "No $(exec) in PATH")))
pwd := $(strip $(shell cd))
else
PREREQS = docker compose code
GPU := $(shell lspci | grep -i "NVIDIA")
	ifneq ($(strip $(GPU)),)
GPU_ID := $(shell nvidia-smi --list-gpus | grep -oP '(\d+)' | head -1)
	endif

K := $(foreach exec,$(PREREQS),\
	$(if $(shell which $(exec)),some string,$(error "No $(exec) in PATH")))
endif

.PHONY: build
build: $(pwd)
#	docker build -t autonomous_img . -f ros2_ws.Dockerfile
ifneq ($(strip $(GPU)),)
	$(info using GPU container)
	cp docker-compose.GPU.yml docker-compose.yml
	sed -i 's/NVIDIA_VISIBLE_DEVICES=.*/NVIDIA_VISIBLE_DEVICES=0/g' docker-compose.yml
	mv docker-compose.yml .devcontainer
#	docker run -d \
#	--gpus all \
#	--env DISPLAY \
#	--env NVIDIA_VISIBLE_DEVICES=$(GPU_ID) \
#	--env NVIDIA_DRIVER_CAPABILITIES=all \
#	--env ROS_DOMAIN_ID=47 \
#	--volume $(pwd):/ws \
#	--network host \
#	--ipc host \
#	--interactive \
#	--tty \
#	--name autonomous \
#	autonomous_img \
#	/bin/bash

else
	$(info using CPU container)
	cp docker-compose.CPU.yml .devcontainer/docker-compose.yml
# 	docker run -d \
# 	--env DISPLAY \
# 	--env ROS_DOMAIN_ID=47 \
# 	--volume $(pwd):/ws \
# 	--network host \
# 	--ipc host \
# 	--interactive \
# 	--tty \
# 	--name autonomous \
# 	autonomous_img \
# 	/bin/bash
endif
	code .