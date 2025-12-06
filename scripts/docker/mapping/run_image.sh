#!/bin/bash

echo "creating container with data-dir $1"

xhost +
docker run \
-it \
--privileged \
--network host \
-v /dev:/dev \
-v /run/udev:/run/udev \
-e DISPLAY=$DISPLAY \
-e PUID=$(id -u) \
-e PGID=$(id -g) \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/ros2_ws/src/livo \
-v /dev:/dev \
-v /mnt:/mnt \
-v /media:/media \
-v $1:/data \
--name mapping-dev \
livo-mapping:dev \
bash
xhost -
