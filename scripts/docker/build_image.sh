#!/bin/bash
# create base image with installation script inside
docker build -f Dockerfile.base -t livo:base .
# perform installation inside the container, this is the only way it works out
docker run -it --name livo-build-ros2 livo:base /bin/bash /install/installation.sh
# commit docker container to image
docker commit livo-build-ros2 livo:ros2
# build the development container
docker build -f Dockerfile --target development -t livo:dev .
