#!/bin/bash

# source ros
source /opt/ros/$ROS_DISTRO/setup.bash
# install necessary build depenencies
apt update && apt install -y --no-install-recommends gnupg neovim byobu mesa-utils
DEBIAN_FRONTEND='noninteractive' apt update && apt install -y -q \
        meson \
        python3-colcon-meson \
        ninja-build \
        pkg-config \
        libyaml-dev \
        python3-yaml \
        python3-ply \
        python3-jinja2 \
        libevent-dev \
        libdrm-dev \
        libcap-dev \
        python3-pip \
        python3-opencv
### Install Libcamera ROS Driver ####
mkdir -p /ros2_ws/src && cd /ros2_ws/src
# Install libcamera and the camera_ros node from source
git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
git clone https://github.com/christianrauch/camera_ros.git
cd /ros2_ws
rosdep update
rosdep install -t exec -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
### NOTE: build will performed for both when livox driver is loaded as well!
### ./build.sh invokes colcon build again!
## colcon build --event-handlers=console_direct+
### Install Livox SDK & ROS driver
mkdir /install && cd /install
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j 8 && make install
cd /ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# NOTE: PCL is required as a dependency by the livox ROS driver!
apt update && apt install -y -q ros-$ROS_DISTRO-pcl*
cd livox_ros_driver2 && ./build.sh humble
# change to ROS2 workspace source
cd /ros2_ws/src
