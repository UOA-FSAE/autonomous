SHELL = /bin/sh
.SUFFIXES:
.SILENT:

PREREQS = docker docker-compose code
ifeq ($(OS),Windows_NT)
	GPU := $(shell wmic path win32_VideoController get name)
	K := $(foreach exec,$(PREREQS),\
        $(if $(shell where $(exec)),some string,$(error "No $(exec) in PATH")))
else
	GPU := lspci | grep -i 'vga\|3d\|2d'
	K := $(foreach exec,$(PREREQS),\
        $(if $(shell which $(exec)),some string,$(error "No $(exec) in PATH")))
endif

.PHONY: build
build:
ifneq ($(findstring NVIDIA,$(GPU)),)
	@echo using GPU container
	docker compose -f docker-compose.GPU.yml up -d
else
	@echo using CPU container
	docker compose -f docker-compose.CPU.yml -d
endif