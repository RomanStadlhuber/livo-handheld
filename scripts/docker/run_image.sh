#!/bin/bash
xhost +
docker run \
--rm \
-it \
--privileged \
--network host \
-v /dev:/dev \
-e DISPLAY=$DISPLAY \
-e PUID=$(id -u) \
-e PGID=$(id -g) \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/catkin_ws/src/livo \
-v /dev:/dev \
-v /mnt:/mnt \
-v /media:/media \
livo:latest \
bash
xhost -
