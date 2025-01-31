#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_RAY_ROS2_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_RAY_ROS2_HH

#include <memory>
#include <complex>
#include <valarray>
#include <chrono>
#include <string>
#include <vector>

// Gazebo headers
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/rendering/GpuLaser.hh>
#include <gazebo/plugins/GazeboRosCameraUtils.hh>
#include <gazebo_ros/node.hpp>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <image_transport/image_transport.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Gazebo ROS utilities
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>



namespace gazebo_ros {

using Complex = std::complex<float>;
using CArray = std::valarray<Complex>;
using CArray2D = std::valarray<CArray>;

using Array = std::valarray<float>;
using Array2D = std::valarray<Array>;


class GazeboRosMultibeamSonar : public gazebo::SensorPlugin 
{
  // constructor
public: GazeboMultibeamSonar();

// destructor
public: ~GazeboMultibeamSonar();

void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

//load plugin
public: void OnNewLaserFrame(const float *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format);

protected: unsigned int width, height, depth;
protected: std::string format;

protected: sensors::GpuRaySensorPtr parentSensor;
protected: rendering::GpuLaserPtr laserCamera;

private: event::ConnectionPtr newLaserFrameConnection;

private: event::ConnectionPtr load_connection_;    

private: rclcpp::Time sensor_update_time_;

/// brief Advertise publishers
public: void Advertise();

/// brief Keep track of number of connections for plugin outputs
private: std::atomic<int> point_cloud_connect_count_{0};
private: std::atomic<int> sonar_image_connect_count_{0};

private: void PointCloudConnect();
private: void PointCloudDisconnect();
private: void SonarImageConnect();
private: void SonarImageDisconnect();

/// brief Compute a normal texture and implement sonar model
private: void UpdatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
private: void ComputeSonarImage();
private: cv::Mat ComputeNormalImage(cv::Mat& depth);
private: double point_cloud_cutoff_;

private: void ComputeCorrector();
private: cv::Mat rand_image;

/// \brief Parameters for sonar properties
private:
    double sonar_freq_;
    double bandwidth_;
    double sound_speed_;
    double max_distance_;
    double source_level_;
    bool const_mu_;
    double absorption_;
    double attenuation_;
    double vertical_fov_;
    
    /// \brief Constant reflectivity
    double mu_;
    bool calculate_reflectivity_;
    cv::Mat reflectivity_image_;
    
    std::vector<float> azimuth_angles_;
    std::vector<float> elevation_angles_;
    std::vector<float> range_vector_;
    std::vector<float> window_;
    std::vector<std::vector<float>> beam_corrector_;
    float beam_corrector_sum_;

    int n_freq_;
    double df_;
    int n_beams_;
    int n_rays_;
    int beam_skips_;
    int ray_skips_;
    int ray_n_azimuth_rays_;
    int ray_n_elevation_rays_;
    
    float plot_scaler_;
    float sensor_gain_;

protected:
    bool debug_flag_;

// ROS2 Components
gazebo_ros::Node::SharedPtr ros_node_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_pub_;
rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_raw_pub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normal_image_pub_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

/// brief Subscriber to VelodyneGpuLaserPointCloud
private:rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_gpu_laser_point_cloud_sub_;

    /// brief A ROS 2 executor that helps process messages
private: rclcpp::CallbackGroup::SharedPtr point_cloud_sub_callback_group_;
    
    /// brief A thread that keeps spinning the ROS 2 node
private: std::thread point_cloud_sub_thread_;
    
    /// rief ROS helper function that processes messages
private: void PointCloudSubThread();

    /// brief Messages for sonar processing
private:sensor_msgs::msg::PointCloud2 point_cloud_msg_;
private:sensor_msgs::msg::Image normal_image_msg_;
private:sensor_msgs::msg::Image sonar_image_msg_;
private:sensor_msgs::msg::Image sonar_image_mono_msg_;

    /// brief ROS 2 does not have marine_acoustic_msgs natively, you may need to define a custom message
private: std::shared_ptr<marine_acoustic_msgs::msg::ProjectedSonarImage> sonar_image_raw_msg_;

    /// brief OpenCV Mat images
private:cv::Mat point_cloud_image_;
private:cv::Mat point_cloud_normal_image_;

    /// brief Topic names
private:std::string point_cloud_topic_name_;
private:std::string sonar_image_raw_topic_name_;
private:std::string sonar_image_topic_name_;

    /// brief CSV log writing stream for verifications
protected:std::ofstream write_log_;
protected: write_counter_;
protected:uint64_t write_number_;
protected:uint64_t write_interval_;
protected:bool write_log_flag_;

};

///////////////////////////////////////////
inline double unnormalized_sinc(double t)
{
    if (t == 0.0)
    {
        return 1.0; // sinc(0) = 1
    }
    return std::sin(t) / t;
}

/// brief A class to store fiducial data
class FiducialData
{
public:
    /// \brief Fiducial ID
    std::string id;

    /// \brief Center point of the fiducial in the image
    ignition::math::Vector2i pt;
};




} // namespace gazebo_ros

#endif