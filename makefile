.PHONY: OS_CHECK GPU_CHECK PREREQ_CHECK INIT start
.SILENT: 

OS_CHECK:
    $(info Checking OS)
    ifeq ($(OS),Windows_NT)
        MACHINE = WIN32
    else
        UNAME_S := $(shell uname -s)
        ifeq ($(UNAME_S),Linux)
            MACHINE = LINUX
        endif
        ifeq ($(UNAME_S),Darwin)
            MACHINE = OSX
        endif
    endif
    $(info OS is $(MACHINE))


GPU_CHECK: OS_CHECK
    $(info Checking presence of NVIDIA GPU)
    ifeq ($(MACHINE),WIN32)
        GPU := $(shell wmic path win32_VideoController get name)
    else
        GPU := $(shell lspci | grep -i 'vga\|3d\|2d')
    endif
    ifeq ($(findstring NVIDIA,$(GPU)),NVIDIA)
        GPU := True
    endif
    $(info NVIDIA GPU found)


PREREQ_CHECK: OS_CHECK
	PREREQS = docker  

	PREREQ_INSTALLED = False


INIT: $(GPU) PREREQ_INSTALLED
	ifeq ($(PREREQ_INSTALLED),True)	
		ifeq ($(GPU), True)
			$(shell docker compose -f docker-compose.GPU.yml -d)
		else
			$(shell docker compose -f docker-compose.CPU.yml -d)
		endif
	endif

	
start:
	$(info Starting autonomous container)
# $(shell docker compose -f docker-compose.CPU.yml up -d)
