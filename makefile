SHELL = /bin/sh
.DEFAULT_GOAL := help
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


.PHONY:help
help:
	@echo "Available targets:"
	@echo " init - initialise the container"
	@echo " start - start the container"

.PHONY: init
init:
ifneq ($(findstring NVIDIA,$(GPU)),)
	@echo GPU
	$(shell docker compose -f docker-compose.GPU.yml -d)
else
	@echo CPU
	$(shell docker compose -f docker-compose.CPU.yml -d)
endif


.PHONY: start
start:
	docker compose -f docker-compose.CPU.yml up -d
