cmake_minimum_required(VERSION 3.8)
project(nps_uw_multibeam_sonar_converted)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Add the plugin library
add_library(p900_sonar_plugin SHARED src/p900_sonar_plugin.cpp)
target_include_directories(p900_sonar_plugin PUBLIC include)
ament_target_dependencies(p900_sonar_plugin rclcpp pluginlib)

# Install headers and library
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS p900_sonar_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#Executables
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node p900_sonar_plugin)
ament_target_dependencies(test_node rclcpp pluginlib)
install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

# Declare plugin metadata
pluginlib_export_plugin_description_file(nps_uw_multibeam_sonar_converted plugins.xml)

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
