#!/bin/bash
xhost +
docker run \
--rm \
-it \
--privileged \
--network host \
-v /dev:/dev \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/catkin_ws/src/livo \
-v /dev:/dev \
-v /mnt:/mnt \
-v /media:/media \
livo:latest \
bash
xhost -
