#include "nps_uw_multibeam_sonar_converted/base_interface.hpp"
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plugin_test_node");

  pluginlib::ClassLoader<nps_uw_multibeam_sonar_converted::BaseInterface> loader(
    "nps_uw_multibeam_sonar_converted", "nps_uw_multibeam_sonar_converted::BaseInterface"
  );

  // Log node startup
  RCLCPP_INFO(node->get_logger(), "🚀 Starting plugin test node");

try {
    RCLCPP_INFO(node->get_logger(), "🔄 Attempting to load plugin...");
    
    auto plugin = loader.createSharedInstance("nps_uw_multibeam_sonar_converted::P900SonarPlugin");
    RCLCPP_INFO(node->get_logger(), "✅ Successfully loaded plugin: 'nps_uw_multibeam_sonar_converted::P900SonarPlugin'");

    // Initialize plugin with logging
    RCLCPP_INFO(node->get_logger(), "🔧 Initializing plugin...");
    plugin->initialize();
    RCLCPP_INFO(node->get_logger(), "⚡ Initialization complete!");

    // Execute plugin with logging
    RCLCPP_INFO(node->get_logger(), "🚦 Executing plugin...");
    plugin->execute();
    RCLCPP_INFO(node->get_logger(), "🎉 Execution completed successfully!");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "❌ Failed to load plugin: %s", e.what());
  }

  RCLCPP_INFO(node->get_logger(), "🛑 Shutting down node");

  rclcpp::shutdown();
  return 0;
}