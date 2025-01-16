FROM ros:jazzy AS base
RUN apt update && apt install -y --no-install-recommends gnupg
# RUN apt update && apt -y upgrade
RUN DEBIAN_FRONTEND='noninteractive' apt install -y --no-install-recommends \
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
	python3-opencv \
     && apt-get clean \
     && apt-get autoremove \
     && rm -rf /var/cache/apt/archives/* \
     && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws/src
# Install libcamera from source
RUN git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
RUN git clone https://github.com/christianrauch/camera_ros.git
RUN /bin/bash -c  "source /opt/ros/$ROS_DISTRO/setup.bash && \
	cd /ros2_ws && \
	rosdep update && \
	rosdep install -t exec -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera && \
	colcon build --event-handlers=console_direct+ && \
	cd /ros2_ws/src"
# Install kmsxx from source
RUN git clone https://github.com/tomba/kmsxx.git
RUN meson setup kmsxx/build kmsxx/
RUN ninja -C kmsxx/build/ install 
# Add the new installations to the python path so that picamera2 can find them
ENV PYTHONPATH $PYTHONPATH/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/app/kmsxx/build/py
# Finally install picamera2 using pip
RUN pip3 install picamera2
# Copy the test script to the container
COPY 3rd/camera_test /ros2/ws/camera_test
# Set the entry point. You can comment this out to use your own test scripts...
CMD ["python3", "/ros2_ws/camera_test/main.py"]
FROM base AS development
WORKDIR /catkin_ws/src
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
