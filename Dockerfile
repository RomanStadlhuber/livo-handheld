# this is the dockerfile that is used to create the base image "livo:ros2"
# which will then be used to create the actual image that is used for development and deployment
FROM ros:humble AS base
WORKDIR /install
# neovim config
COPY init.vim /root/.config/nvim/init.vim
# set shell to bash
SHELL ["/bin/bash", "-c"]
# install necessary build depenencies
RUN apt-get update && apt-get install -y --no-install-recommends gnupg neovim byobu mesa-utils
RUN DEBIAN_FRONTEND='noninteractive' apt-get update && apt-get install -y -q \
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
WORKDIR /ros2_ws/src
# Install libcamera and the camera_ros node from source
# NOTE: libcamera needs to be checked out here
# to be compatible with the meson & ninja versions that can be loaded from this image
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd /ros2_ws/src/libcamera && git checkout 6ddd79b && cd .. && \
    git clone https://github.com/christianrauch/camera_ros.git
WORKDIR /ros2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && \ 
    rosdep install -t exec -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
### NOTE: build will performed for both when livox driver is loaded as well!
### ./build.sh invokes colcon build again!
## colcon build --event-handlers=console_direct+
### Install Livox SDK & ROS driver
WORKDIR /install
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
RUN cd /install/Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j 8 && make install
WORKDIR /ros2_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# NOTE: PCL is required as a dependency by the livox ROS driver!
RUN apt-get update && apt-get install -y -q ros-$ROS_DISTRO-pcl*
WORKDIR /ros2_ws/src/livox_ros_driver2
RUN ./build.sh humble


# change to ROS2 workspace source
FROM base AS development
WORKDIR /ros2_ws/src
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
# copy to a global path so it is always available in both dev and deploy
COPY recorder_runner/launch/MID360_config.json /MID360_config.json
WORKDIR /ros2_ws
FROM development AS deploy
COPY recorder_runner /ros2_ws/src/recorder_runner
SHELL ["/bin/bash", "-c"]
# build recorder_runner
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&  \
    source /ros2_ws/install/setup.bash && \
    source /ros2_ws/src/recorder_runner/build.sh
COPY webinterface /app
WORKDIR /app
# instal webapp dependencies
RUN pip3 install --no-cache-dir -r requirements.txt
# run the webapp
COPY scripts/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]