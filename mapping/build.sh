#! /bin/bash
ORIG_PTH=$(pwd)
cd /ros2_ws
# linker explicitly needs path to Open3D shared object files
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/install/open3d-devel/lib
colcon build --symlink-install --packages-select mapping --cmake-args -DCMAKE_BUILD_TYPE=Release
cd $ORIG_PTH