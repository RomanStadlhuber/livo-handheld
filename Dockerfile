FROM arm32v7/ros:noetic-perception-focal as base
ENV DEBIAN_FRONTEND noninteractive

# install locales
RUN apt-get update
RUN apt-get install apt-utils wget locales -y

# Set the locale
RUN locale-gen en_US.UTF-8
RUN update-locale LANG=en_US.UTF-8

# set linked library path
ENV LD_LIBRARY_PATH /usr/local/lib

# create root catkin workspace
WORKDIR /catkin_ws

# install utilities
# NOTE: mesa-utils is for GUI testing
RUN apt-get update && apt-get install -y -q curl git byobu neovim mesa-utils \
    python-is-python3 python3-pip python3-venv python3-rpi.gpio

# install raspicam_node for ROS noetic
WORKDIR /catkin_ws/src
# add repositories required to install raspicam_node dependencies (next step)

# RUN sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list' && \
#    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8 && \
#    apt-get update

# see: yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
RUN apt install -y -q libraspberrypi0 libraspberrypi-dev && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/UbiquityRobotics/raspicam_node.git -b noetic-devel 
RUN git clone https://github.com/RomanStadlhuber/nxp_precision_9dof.git
# install MID360 support (from https://github.com/RomanStadlhuber/mid360_docker/blob/main/Dockerfile)
# install Livox-SDK2 which is a dependency for the ROS driver
WORKDIR /install
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd ./Livox-SDK2 && \
    mkdir build && cd build && cmake .. && make -j && make install
# make VTK (https://docs.vtk.org/en/latest/index.html) symlink to version-less folder
RUN ln -s /usr/bin/vtk7 /usr/bin/vtk
# install mid360 driver
WORKDIR /catkin_ws/src
# clone the driver repo
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && /catkin_ws/src/livox_ros_driver2/build.sh ROS1"
# build the ROS workspace
WORKDIR /catkin_ws
# install some dependencies that are required to build
RUN apt-get update && apt-get install -y -q ros-noetic-diagnostic-updater
# build and set up all packages (IMU and camera)
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make" 
RUN chmod +x /catkin_ws/src/nxp_precision_9dof/scripts/*.py && \
    pip install -r /catkin_ws/src/nxp_precision_9dof/requirements.txt
FROM base as development
WORKDIR catkin_ws/src
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN apt-get update && apt-get install -y -q ros-noetic-rqt-image-view
# copy neovim configuration file
COPY init.vim /root/.config/nvim/init.vim

# optionally, use a deployment stage (e.g. for usage via docker-compose)
# FROM development as deploy
