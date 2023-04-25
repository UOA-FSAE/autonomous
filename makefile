.SUFFIXES:
# .SILENT:


ifeq ($(OS),Windows_NT)
SHELL := C:/Program Files/Git/bin/bash.exe
PREREQS = docker docker-compose code
GPU := wmic path win32_VideoController get name | findstr "NVIDIA"
K := $(foreach exec,$(PREREQS),\
	$(if $(shell where $(exec)),some string,$(error "No $(exec) in PATH")))
pwd := $(strip $(shell cd))
else
PREREQS = docker compose code
GPU := lspci | grep -i 'vga\|3d\|2d'
K := $(foreach exec,$(PREREQS),\
	$(if $(shell which $(exec)),some string,$(error "No $(exec) in PATH")))
endif


.PHONY: build
build:
	docker build -t autonomous_test . -f ros2_ws.Dockerfile
ifneq ($(findstring NVIDIA, $(GPU)),)
	ifeq ($(OS),Windows_NT)
GPU_ID := $(shell nvidia-smi --query-gpu=index --format=csv,noheader | findstr "^[0-9]")
	else
GPU_ID := $(shell nvidia-smi --list-gpus | grep -oP '(\d+)' | head -1)
	endif

	@echo using GPU container
	docker run -d \
	--gpus all \
	--env DISPLAY \
	--env NVIDIA_VISIBLE_DEVICES=$(GPU_ID) \
	--env NVIDIA_DRIVER_CAPABILITIES=all \
	--env ROS_DOMAIN_ID=47 \
	--volume $(pwd):/ws \
	--network host \
	--ipc host \
	--interactive \
	--tty \
	autonomous_test \
	/bin/bash

else

	@echo using CPU container
	docker run -d \
	--env DISPLAY \
	--env ROS_DOMAIN_ID=47 \
	--volume $(pwd):/ws \
	--network host \
	--ipc host \
	--interactive \
	--tty \
	autonomous_test \
	/bin/bash
endif