#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash
cd /app
gunicorn -w 4 -b 0.0.0.0:8000 app:app
