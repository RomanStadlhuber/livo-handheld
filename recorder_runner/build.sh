#!/bin/bash
RUNDIR=$(pwd)
cd /ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash
colcon build --packages-select recorder_runner
cd $RUNDIR
