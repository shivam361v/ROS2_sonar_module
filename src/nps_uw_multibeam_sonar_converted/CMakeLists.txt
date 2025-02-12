cmake_minimum_required(VERSION 3.8)
project(nps_uw_multibeam_sonar_converted)

# Enable CUDA
set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA CXX)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(image_transport REQUIRED)


# Set CUDA Architecture (Modify based on GPU Compute Capability)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")

# Include headers and libraries
include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)


link_directories(${CUDA_LIBRARY_DIRS})

# Add the plugin library
add_library(gazebo_multibeam_sonar_raster_based SHARED 
src/gazebo_multibeam_sonar_raster_based.cpp
src/sonar_calculation_cuda.cuh
include/nps_uw_multibeam_sonar_converted/sonar_calculation_cuda.cuh
)

set_target_properties(gazebo_multibeam_sonar_raster_based
                      PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

                    

target_include_directories(gazebo_multibeam_sonar_raster_based PUBLIC include)

# Link CUDA Libraries
target_link_libraries(gazebo_multibeam_sonar_raster_based 
    ${CUDA_LIBRARIES}
    ${CUDA_CUFFT_LIBRARIES}
)

ament_target_dependencies(gazebo_multibeam_sonar_raster_based 
rclcpp 
pluginlib 
sensor_msgs 
std_msgs 
OpenCV
cv_bridge
pcl_ros
marine_acoustic_msgs
gazebo_ros
)

# Install headers and library
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS gazebo_multibeam_sonar_raster_based
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executable (only compile test_node.cpp if plugin is built as a library)
add_executable(test_node 
src/test_node.cpp
)

target_link_libraries(test_node 
    gazebo_multibeam_sonar_raster_based 
    ${CUDA_LIBRARIES}
    ${CUDA_CUFFT_LIBRARIES}
)

target_link_libraries(test_node gazebo_multibeam_sonar_raster_based 
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(test_node
  rclcpp 
  sensor_msgs
  std_msgs
  marine_acoustic_msgs
  gazebo_ros
  OpenCV
  cv_bridge
  pcl_ros
  pcl_conversions
  pluginlib
  image_transport
)
install(TARGETS gazebo_multibeam_sonar_raster_based test_node DESTINATION lib/${PROJECT_NAME})

# Declare plugin metadata
pluginlib_export_plugin_description_file(nps_uw_multibeam_sonar_converted plugins.xml)

ament_package()



