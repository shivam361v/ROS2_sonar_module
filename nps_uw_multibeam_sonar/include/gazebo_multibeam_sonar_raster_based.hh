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
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/msgs/msgs.hh>

// Standard includes
#include <memory>
#include <string>
#include <vector>

namespace gazebo_ros_multibeam_sonar {

class MultibeamSonarPlugin : public gazebo::ModelPlugin {
public:
  MultibeamSonarPlugin();
  ~MultibeamSonarPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  // ROS2 Node
  rclcpp::Node::SharedPtr ros_node_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Gazebo Transport Node
  gazebo::transport::NodePtr gazebo_node_;

  // Timer for publishing data
  rclcpp::TimerBase::SharedPtr timer_;

  // Helper function to publish data
  void PublishData();

  // Parameters
  std::string namespace_;
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
void MultibeamSonarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  RCLCPP_INFO(rclcpp::get_logger("MultibeamSonarPlugin"), "Loading plugin with model: %s", model->GetName().c_str());

  // Check if ROS2 node is initialized
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("MultibeamSonarPlugin"), "ROS2 is not initialized. Plugin will not function correctly.");
    return;
  }

  // Initialize ROS2 node
  ros_node_ = rclcpp::Node::make_shared("multibeam_sonar_plugin");
  RCLCPP_INFO(ros_node_->get_logger(), "ROS2 node initialized");

  // Initialize publishers
  pointcloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>("image", 10);
  camera_info_pub_ = ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

  RCLCPP_INFO(ros_node_->get_logger(), "Publishers initialized for topics: pointcloud, image, camera_info");

  // Log parameters
  RCLCPP_INFO(ros_node_->get_logger(), "Namespace: %s", namespace_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "PointCloud Topic: %s", pointcloud_topic_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "Image Topic: %s", image_topic_.c_str());
  RCLCPP_INFO(ros_node_->get_logger(), "CameraInfo Topic: %s", camera_info_topic_.c_str());
}

// Implementation of PublishData function
void MultibeamSonarPlugin::PublishData() {
  RCLCPP_DEBUG(ros_node_->get_logger(), "Publishing data...");
  // Placeholder: Add data publishing logic here
}

} // namespace gazebo_ros_multibeam_sonar

#endif // GAZEBO_ROS_MULTIBEAM_SONAR_HH
