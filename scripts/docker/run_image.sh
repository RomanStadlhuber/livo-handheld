#!/bin/bash
docker run \
--rm \
-it \
--privileged \
--network host \
-v /dev:/dev \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/catkin_ws/src/livo \
-v /mnt:/mnt \
-v /media:/media \
livo:latest \
bash
