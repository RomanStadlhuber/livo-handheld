cmake_minimum_required(VERSION 3.16.0)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# NOTE: remove this to enable Open3D based visualization
# (should be disabled for the ROS node)
add_definitions(-DDISABLEVIZ)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/install/open3d-devel/lib/cmake")
find_package(Open3D REQUIRED)
find_package(open3d_conversions REQUIRED) # for converting to ROS PointCloud2
find_package(config_utilities REQUIRED)


# set(CMAKE_BUILD_TYPE Release)

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
  # src/mapping/factors/PlanarClusterFactor.cpp
  src/mapping/factors/PointToPlaneFactor.cpp
  src/mapping/Visualization.cpp
  src/mapping/MappingSystem.cpp
  # common logic state
  src/mapping/Buffers.cpp
  src/mapping/States.cpp
  # frontends
  src/mapping/frontend/ImuFrontend.cpp
  src/mapping/frontend/LidarFrontend.cpp
  # backend
  src/mapping/backend/FeatureManager.cpp
  src/mapping/backend/Smoother.cpp
)

target_include_directories(mapping PUBLIC include)

target_link_libraries(mapping
  Eigen3::Eigen 
  Open3D::Open3D 
  gtsam 
  gtsam_unstable 
  config_utilities::config_utilities
)

add_executable(mapper src/mapper_node.cpp)
ament_target_dependencies(mapper 
  rclcpp std_msgs geometry_msgs sensor_msgs
  livox_ros_driver2 rosbag2_cpp rosbag2_storage
  tf2 tf2_ros nav_msgs open3d_conversions
  visualization_msgs
)
target_link_libraries(mapper mapping)

install(TARGETS
  mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- Documentation ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
  include(FetchContent)
  FetchContent_Declare(doxygen-awesome-css
    URL https://github.com/jothepro/doxygen-awesome-css/archive/refs/heads/main.zip
  )
  FetchContent_MakeAvailable(doxygen-awesome-css)
  FetchContent_GetProperties(doxygen-awesome-css SOURCE_DIR AWESOME_CSS_DIR)

  set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
  set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

  add_custom_target(docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating documentation with Doxygen"
  )
endif()

ament_package()
