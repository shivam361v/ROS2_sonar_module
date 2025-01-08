/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_HH

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>

// Standard includes
#include <memory>
#include <string>
#include <vector>

namespace gazebo_ros_multibeam_sonar {

class MultibeamSonarPlugin : public gazebo::SensorPlugin {
public:
  MultibeamSonarPlugin();
  ~MultibeamSonarPlugin();

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  // ROS2 Node
  rclcpp::Node::SharedPtr ros_node_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Gazebo Depth Camera Sensor
  gazebo::sensors::DepthCameraSensorPtr depth_camera_sensor_;

  // Timer for publishing data
  rclcpp::TimerBase::SharedPtr timer_;

  // Helper function to publish data
  void PublishData();

  // Parameters
  std::string pointcloud_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
};

// Implementation of constructor
MultibeamSonarPlugin::MultibeamSonarPlugin() {
  RCLCPP_INFO(rclcpp::get_logger("MultibeamSonarPlugin"), "Constructing MultibeamSonarPlugin instance");
}

// Implementation of destructor
MultibeamSonarPlugin::~MultibeamSonarPlugin() {
  RCLCPP_INFO(rclcpp::get_logger("MultibeamSonarPlugin"), "Destroying MultibeamSonarPlugin instance");
}

// Implementation of Load function
void MultibeamSonarPlugin::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  RCLCPP_INFO(rclcpp::get_logger("MultibeamSonarPlugin"), "Loading plugin with sensor: %s", sensor->Name().c_str());

  // Initialize ROS2 node
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("MultibeamSonarPlugin"), "ROS2 is not initialized. Plugin will not function correctly.");
    return;
  }

  ros_node_ = rclcpp::Node::make_shared("multibeam_sonar_plugin");
  RCLCPP_INFO(ros_node_->get_logger(), "ROS2 node initialized");

  // Initialize depth camera sensor
  depth_camera_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(sensor);
  if (!depth_camera_sensor_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Sensor is not a DepthCameraSensor");
    return;
  }

  // Read parameters from SDF
  pointcloud_topic_ = sdf->Get<std::string>("pointcloud_topic", "pointcloud").first;
  image_topic_ = sdf->Get<std::string>("image_topic", "image").first;
  camera_info_topic_ = sdf->Get<std::string>("camera_info_topic", "camera_info").first;

  // Initialize publishers
  pointcloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
  image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
  camera_info_pub_ = ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);

  RCLCPP_INFO(ros_node_->get_logger(), "Publishers initialized for topics: %s, %s, %s", pointcloud_topic_.c_str(), image_topic_.c_str(), camera_info_topic_.c_str());

  // Create a timer to periodically publish data
  timer_ = ros_node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MultibeamSonarPlugin::PublishData, this));
}

// Implementation of PublishData function
void MultibeamSonarPlugin::PublishData() {
  if (!depth_camera_sensor_) {
    RCLCPP_WARN(ros_node_->get_logger(), "DepthCameraSensor is not initialized");
    return;
  }

  RCLCPP_DEBUG(ros_node_->get_logger(), "Publishing data...");

  // Example publishing logic (replace with actual data handling)
  auto pointcloud_msg = sensor_msgs::msg::PointCloud2();
  pointcloud_pub_->publish(pointcloud_msg);

  auto image_msg = sensor_msgs::msg::Image();
  image_pub_->publish(image_msg);

  auto camera_info_msg = sensor_msgs::msg::CameraInfo();
  camera_info_pub_->publish(camera_info_msg);
}

} // namespace gazebo_ros_multibeam_sonar

#endif // GAZEBO_ROS_MULTIBEAM_SONAR_HH
