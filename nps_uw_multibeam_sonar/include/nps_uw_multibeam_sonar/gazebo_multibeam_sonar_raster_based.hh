#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_HH

// ROS2 stuff
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/callback_group.hpp>

// ROS2 messages
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>

// OpenCV
#include <opencv2/core.hpp>

// C++ standard libraries
#include <complex>
#include <valarray>
#include <sstream>
#include <chrono>
#include <string>

// Gazebo libraries
#include <sdf/param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/time.hh>
#include <gazebo/sensors/sensor.hh>
#include <gazebo/sensors/sensor_types.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>

// For variational reflectivity
#include <sdf/element.hh>
#include <gazebo/rendering/scene.hh>
#include <gazebo/rendering/visual.hh>
#include "selection_buffer/selection_buffer.hh"