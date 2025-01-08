#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <memory>
#include <string>
#include <vector>
#include <limits>

namespace gazebo
{
  class NpsGazeboRos2MultibeamSonar : public DepthCameraPlugin
  {
  public:
    NpsGazeboRos2MultibeamSonar()
      : Node("nps_gazebo_ros2_multibeam_sonar")
    {
    }

    virtual ~NpsGazeboRos2MultibeamSonar() {}

    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "ROS2 is not running");
        return;
      }

      this->sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
      if (!this->sensor_)
      {
        RCLCPP_ERROR(this->get_logger(), "Plugin not attached to a DepthCameraSensor");
        return;
      }

      this->depth_camera_ = this->sensor_->DepthCamera();
      if (!this->depth_camera_)
      {
        RCLCPP_ERROR(this->get_logger(), "DepthCamera not found");
        return;
      }

      this->depth_camera_->SetActive(true);

      // Initialize ROS2 publishers
      depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 10);
      sonar_image_pub_ = this->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>("sonar_image_raw", 10);

      // Connect Gazebo update events
      new_depth_frame_connection_ = this->depth_camera_->ConnectNewDepthFrame(
        std::bind(&NpsGazeboRos2MultibeamSonar::OnNewDepthFrame, this, std::placeholders::_1, 
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

      RCLCPP_INFO(this->get_logger(), "NPS Gazebo ROS2 Multibeam Sonar Plugin loaded");
    }

  private:
    void OnNewDepthFrame(const float* image, unsigned int width, unsigned int height, unsigned int depth, const std::string& format)
    {
      if (!rclcpp::ok())
        return;

      // Create and publish depth image message
      auto depth_msg = std::make_shared<sensor_msgs::msg::Image>();
      depth_msg->header.stamp = this->now();
      depth_msg->header.frame_id = "sonar_frame";
      depth_msg->width = width;
      depth_msg->height = height;
      depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_msg->step = width * sizeof(float);
      depth_msg->data.resize(width * height * sizeof(float));
      memcpy(depth_msg->data.data(), image, width * height * sizeof(float));
      depth_image_pub_->publish(*depth_msg);

      // Simulate sonar image generation (simplified example)
      auto sonar_msg = std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>();
      sonar_msg->header = depth_msg->header;
      sonar_msg->ping_info.frequency = 900e3;
      sonar_msg->ping_info.sound_speed = 1500.0;
      sonar_image_pub_->publish(*sonar_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_image_pub_;

    sensors::DepthCameraSensorPtr sensor_;
    rendering::DepthCameraPtr depth_camera_;

    event::ConnectionPtr new_depth_frame_connection_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRos2MultibeamSonar)
}
