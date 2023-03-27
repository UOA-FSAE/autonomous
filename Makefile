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

.PHONY: build
build:
	docker build -t $(ROS2_IMAGE_NAME) .

.PHONY: up
upp:
	docker-compose -f ./docker-compose.yml -p local_run run autonomous_ros2

.PHONY: up
up:
	docker-compose -f ./docker-compose_macos.yml -p local_run run autonomous_ros2

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