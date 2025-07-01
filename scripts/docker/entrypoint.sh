#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash
cd /app
# IMPORTANT: the web application CANNOT handle multiple workers,
# since references to the ROS2 launch processes are handled as globals
# therefore, the number of workers MUST be set to 1
gunicorn -w 1 -b 0.0.0.0:8000 app:app
