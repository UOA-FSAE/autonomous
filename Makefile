all:
	@echo "usage: make [COMMAND]"
	@echo
	@echo "COMMAND options:"
	@echo "    help"
	@echo "        - show this message"
	@echo "    compose [SERVICE]"
	@echo "        - run docker-compose up and run required services"
	@echo
	@echo "SERVICE options:"
	@echo "    [blank]"
	@echo "        - run simple on developer machine"

help: all

ROS2_IMAGE_NAME = autonomous

ifeq ($(OS),Windows_NT) 
    detected_OS := Windows
else
    detected_OS := $(shell sh -c 'uname 2>/dev/null || echo Unknown')
endif

# ifeq ($(detected_OS),Windows)
#     echo 1
# endif
# ifeq ($(detected_OS),Darwin)        # Mac OS X
#     @echo "hello"
# endif
# ifeq ($(detected_OS),Linux)
#     echo 3
# endif

.PHONY: build
# make build target=jetson
build:
	docker-compose -f ./docker-compose-$(detected_OS).yml -p local_run build $(target)

.PHONY: upp
up:
	docker-compose -f ./docker-compose-$(detected_OS).yml -p local_run run $(target)

.PHONY: down
down:
	docker-compose down

.PHONY: talker
talker:
	docker exec -it $(shell docker ps -qf "name=autonomous_ros2-1") source /opt/ros/humble/setup.bash
	docker exec -it $(shell docker ps -qf "name=autonomous_ros2-1") ros2 run demo_nodes_cpp talker

.PHONY: listener
listener:
	docker exec -it $(shell docker ps -qf "name=ros2_ros2_1") ros2 run demo_nodes_cpp listener