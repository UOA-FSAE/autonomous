# autonomous

## Setting up docker container:

1. Install docker desktop, if you install docker standalone (eg brew install docker) it might not come with the compose plugin.
    
    [https://docs.docker.com/compose/install/](https://docs.docker.com/compose/install/)
    
2. 

```bash
git clone ...
cd autonomous
```

3. If macos/linux replace docker-compose.yml with:

```yaml
version: '3.4'

services:
  autonomous_ros2:
    platform: linux/amd64/v8
    image: autonomous

    build:
      context: .
      dockerfile: ./ros2_ws.Dockerfile

    environment:
      - DISPLAY
      
      - ROS_DOMAIN_ID=47  # Its 47 for obvious reasons

    volumes:
      - .:/ws

    
    network_mode: host
    ipc: host
    stdin_open: true
    tty: true
```

4. Attach container

```bash
docker-compose up
```

5. use bash inside the container

```bash
docker exec -it CONTAINER_NAME bash
# get container name using `docker ps`

source /opt/ros/humble/setup.bash
```
