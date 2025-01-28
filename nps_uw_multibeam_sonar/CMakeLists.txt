cmake_minimum_required(VERSION 3.8)
project(nps_uw_multibeam_sonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ensure C++17 is enabled
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# CUDA settings
include_directories(${CUDA_INCLUDE_DIRS})
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")
set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${marine_acoustic_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Plugin libraries
add_library(nps_multibeam_sonar_ros_plugin SHARED
  src/gazebo_multibeam_sonar_raster_based.cpp
  src/sonar_calculation_cuda.cu
  src/MaterialSwitcher.cc
  src/SelectionBuffer.cc
  src/SelectionRenderListener.cc
)

ament_target_dependencies(nps_multibeam_sonar_ros_plugin
  rclcpp
  gazebo_ros
  tf2
  cv_bridge
  geometry_msgs
  OpenCV
  marine_acoustic_msgs
  PCL
)

target_link_libraries(nps_multibeam_sonar_ros_plugin
  ${GAZEBO_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${CUDA_CUFFT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(nps_multibeam_sonar_ray_ros_plugin SHARED
  src/gazebo_multibeam_sonar_ray_based.cpp
  src/sonar_calculation_cuda.cu
  src/MaterialSwitcher.cc
  src/SelectionBuffer.cc
  src/SelectionRenderListener.cc
)

set_target_properties(nps_multibeam_sonar_ros_plugin
  PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

ament_target_dependencies(nps_multibeam_sonar_ray_ros_plugin
  rclcpp
  gazebo_ros
  tf2
  cv_bridge
  geometry_msgs
  OpenCV
  marine_acoustic_msgs
  PCL
  pluginlib
)

target_link_libraries(nps_multibeam_sonar_ray_ros_plugin
  ${GAZEBO_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${marine_acoustic_msgs_LIBRARIES}
  ${CUDA_CUFFT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install plugins
install(
  TARGETS nps_multibeam_sonar_ros_plugin nps_multibeam_sonar_ray_ros_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, worlds, urdf, and models
install(DIRECTORY
  launch
  worlds
  urdf
  models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

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

ament_package()
