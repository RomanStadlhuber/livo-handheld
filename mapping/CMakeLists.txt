cmake_minimum_required(VERSION 3.16.0)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/install/open3d-devel/lib/cmake")
find_package(Open3D REQUIRED)

# remove this when done with debugging
message(WARNING "Building in Debug mode")
set(CMAKE_BUILD_TYPE Debug)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(mapping
  src/mapping/factors/PointToPlaneFactor.cpp
  src/mapping/MappingSystem.cpp
)

target_include_directories(mapping PUBLIC include)

target_link_libraries(mapping
  Eigen3::Eigen Open3D::Open3D gtsam gtsam_unstable
)

add_executable(mapper src/mapper_node.cpp)
ament_target_dependencies(mapper 
  rclcpp std_msgs geometry_msgs sensor_msgs
  livox_ros_driver2 rosbag2_cpp rosbag2_storage
)
target_link_libraries(mapper mapping)

install(TARGETS
  mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
