#! /bin/bash
$ORIG_PTH = $cwd()
cd /ros2_ws
colcon build --symlink-install --packages-select mapping
cd $ORIG_PTH