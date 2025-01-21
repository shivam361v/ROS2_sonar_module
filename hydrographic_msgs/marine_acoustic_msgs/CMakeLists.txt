cmake_minimum_required(VERSION 3.8)
project(marine_acoustic_msgs)

# Package name
get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")

project(${PACKAGE_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Message files
file(
  GLOB rosMsgFiles
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
  "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

if(rosMsgFiles)
  foreach(rosMsg ${rosMsgFiles})
    message(STATUS "Processing message file: ${rosMsg}")
  endforeach(rosMsg)
else()
  message(WARNING "No message files found in the msg directory!")
endif()

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${rosMsgFiles}
  DEPENDENCIES geometry_msgs std_msgs
)

# Install
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(geometry_msgs std_msgs)


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
