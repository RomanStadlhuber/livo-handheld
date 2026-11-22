#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $(basename "$0") <data-dir>" >&2
    exit 1
fi

echo "creating container with data-dir $1"

# forward the host ssh-agent (if any) so git pull / push over SSH work inside the container
# NOTE: resolve symlinks first (e.g. byobu's ~/.byobu/.ssh-agent), then mount the socket's directory
SSH_ARGS=()
if [ -n "$SSH_AUTH_SOCK" ]; then
    SOCK=$(readlink -f "$SSH_AUTH_SOCK")
    SSH_ARGS=(-v "$(dirname "$SOCK")":/ssh-agent -e SSH_AUTH_SOCK=/ssh-agent/"$(basename "$SOCK")")
    echo "forwarding ssh-agent socket $SOCK"
else
    echo "no ssh-agent found (SSH_AUTH_SOCK unset), git pull / push over SSH will not work in the container"
fi

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
"${SSH_ARGS[@]}" \
--name mapping-dev \
livo-mapping:dev \
bash
xhost -
