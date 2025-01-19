#!/bin/bash
RUNDIR=$(pwd)
cd /ros2_ws
colcon_build --packages-select recorder_runner
cd $RUNDIR
