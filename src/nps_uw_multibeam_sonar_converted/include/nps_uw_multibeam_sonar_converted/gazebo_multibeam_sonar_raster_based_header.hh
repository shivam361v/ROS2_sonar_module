#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_HH

// ROS 2 stuff
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/callback_group.hpp>

// ROS 2 message headers
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
// #include <image_transport/image_transport/image_transport.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// Gazebo Classic
#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

// OpenCV and PCL
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// For variational reflectivity
#include "selection_buffer/SelectionBuffer.hh"

namespace gazebo
{
    typedef std::complex<float> Complex;
    typedef std::valarray<Complex> CArray;
    typedef std::valarray<CArray> CArray2D;

    typedef std::valarray<float> Array;
    typedef std::valarray<Array> Array2D;


class NpsGazeboRosMultibeamSonar : public SensorPlugin
{

    public:
    /// \brief Constructor
    NpsGazeboRosMultibeamSonar();

    /// \brief Destructor
    ~NpsGazeboRosMultibeamSonar() override;

    /// \brief Load the plugin, taking the SDF root element.
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) override;

    /// \brief Advertise point cloud, depth image, and other topics.
    virtual void Advertise();

    /// \brief Helper function to populate fiducials (if none are specified in SDF)
    void PopulateFiducials();

    // Public members for further processing:
    /// \brief Selection buffer used for occlusion detection
    std::unique_ptr<gazebo::rendering::SelectionBuffer> selectionBuffer;

    /// \brief Pointer to the Gazebo scene
    gazebo::rendering::ScenePtr scene;

    /// \brief If true, detect all objects in the world.
    bool detectAll = false;

    /// \brief Set of fiducial IDs tracked by this camera.
    std::set<std::string> fiducials;

  protected:
    /// \brief Update callback for new depth frames.
    virtual void OnNewDepthFrame(const float *_image,
                                 unsigned int _width,
                                 unsigned int _height,
                                 unsigned int _depth,
                                 const std::string &_format);

    /// \brief Update callback for new image frames.
    virtual void OnNewImageFrame(const unsigned char *_image,
                                 unsigned int _width,
                                 unsigned int _height,
                                 unsigned int _depth,
                                 const std::string &_format);

    /// \brief Publish camera info (you can implement your own version here)\n"
    virtual void PublishCameraInfo();

  private:
    /// \brief Compute a normal texture and implement the sonar model.
    void ComputeSonarImage(const float *_src);

    /// \brief Compute the point cloud from depth data.
    void ComputePointCloud(const float *_src);

    /// \brief Compute the incidence angle given azimuth, elevation, and surface normal.
    double ComputeIncidence(double azimuth, double elevation, cv::Vec3f normal);

    /// \brief Compute a normal image from a depth image.
    cv::Mat ComputeNormalImage(cv::Mat &depth);

    /// \brief Compute any correction factors required by the sonar model.
    void ComputeCorrector();

    /// \brief A random image for sonar noise simulation.
    cv::Mat rand_image;

    // Sonar property parameters
    double sonarFreq;
    double bandwidth;
    double soundSpeed;
    double maxDistance;
    double sourceLevel;
    bool constMu;
    bool customTag;
    double absorption;
    double attenuation;
    double verticalFOV;
    double mu;
    physics::WorldPtr world;
    std::string reflectivityDatabaseFileName;
    std::string reflectivityDatabaseFilePath;
    std::string customTagDatabaseFileName;
    std::string customTagDatabaseFilePath;
    std::vector<std::string> objectNames;
    std::vector<float> reflectivities;
    double biofouling_rating_coeff;
    double roughness_coeff;
    double maxDepth, maxDepth_before, maxDepth_beforebefore;
    double maxDepth_prev;
    bool calculateReflectivity;
    bool artificialVehicleVibration;
    cv::Mat reflectivityImage;
    float* rangeVector;
    float* window;
    float** beamCorrector;
    float beamCorrectorSum;
    int nFreq;
    double df;
    int nBeams;
    int nRays;
    int beamSkips;
    int raySkips;
    int ray_nAzimuthRays;
    int ray_nElevationRays;
    float* elevation_angles;
    float plotScaler;
    float sensorGain;
    bool debugFlag;

    /// \brief CSV log writing stream for verifications.
    std::ofstream writeLog;
    u_int64_t writeCounter;
    u_int64_t writeNumber;
    u_int64_t writeInterval;
    bool writeLogFlag;

    // Connection counters and connection functions for plugin outputs.
    int depth_image_connect_count_;
    int depth_info_connect_count_;
    int point_cloud_connect_count_;
    int sonar_image_connect_count_;
    void DepthImageConnect();
    void DepthImageDisconnect();
    void DepthInfoConnect();
    void DepthInfoDisconnect();
    void NormalImageConnect();
    void NormalImageDisconnect();
    void PointCloudConnect();
    void PointCloudDisconnect();
    rclcpp::Time last_depth_image_camera_info_update_time_;

    // ROS 2 publishers for depth image, normal image, point cloud, sonar images, etc.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normal_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_image_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_pub_;

    sensor_msgs::msg::Image depth_image_msg_;
    sensor_msgs::msg::Image normal_image_msg_;
    sensor_msgs::msg::PointCloud2 point_cloud_msg_;
    marine_acoustic_msgs::msg::ProjectedSonarImage sonar_image_raw_msg_;
    sensor_msgs::msg::Image sonar_image_msg_;
    sensor_msgs::msg::Image sonar_image_mono_msg_;
    cv::Mat point_cloud_image_;

    // A random number generator for noise simulation.
    std::default_random_engine generator;

    // Note: In ROS 2, GazeboRosCameraUtils is not ported, so its functionality must be reimplemented.\n"
    // For example, you can implement your own PublishCameraInfo() using the camera info messages and parameters.
    // using GazeboRosCameraUtils::PublishCameraInfo; // Removed because it's not available in ROS 2

    // Topic names for the various outputs
    std::string depth_image_topic_name_;
    std::string depth_image_camera_info_topic_name_;
    std::string point_cloud_topic_name_;
    std::string sonar_image_raw_topic_name_;
    std::string sonar_image_topic_name_;

    double point_cloud_cutoff_;

    // Overload with our own time stamp management for depth sensor update.
    rclcpp::Time depth_sensor_update_time_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_image_camera_info_pub_;

    event::ConnectionPtr load_connection_;

    // From DepthCameraPlugin
    unsigned int width, height, depth;
    std::string format;

    sensors::DepthCameraSensorPtr parentSensor;
    rendering::DepthCameraPtr depthCamera;

    event::ConnectionPtr newDepthFrameConnection_;
    event::ConnectionPtr newImageFrameConnection_;
    event::ConnectionPtr newRGBPointCloudConnection_;

    // A couple of "convenience" functions for computing azimuth & elevation
private: inline double Azimuth(int col)
{
  // Get the horizontal field of view in radians from the depth camera.
  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  // Compute the focal length based on the image width and hfov.
  double fl = static_cast<double>(this->width) / (2.0 * std::tan(hfov / 2.0));
  double azimuth = 0.0;
  if (this->width > 1)
    azimuth = std::atan2(static_cast<double>(col) - 0.5 * static_cast<double>(this->width - 1), fl);
  return azimuth;
}

private: inline double Elevation(int row)
{
  // Get the horizontal field of view in radians (assuming similar handling for vertical FOV).\n"
  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = static_cast<double>(this->width) / (2.0 * std::tan(hfov / 2.0));
  double elevation = 0.0;
  if (this->height > 1)
    elevation = std::atan2(static_cast<double>(row) - 0.5 * static_cast<double>(this->height - 1), fl);
  return elevation;
}

};



///////////////////////////////////////////
// Simplified unnormalized_sinc function (removing the try-catch)
inline double unnormalized_sinc(double t)
{
  // Check if t is zero to avoid division by zero
  return (t == 0.0) ? 1.0 : std::sin(t) / t;
}

/// \brief A class to store fiducial data
class FiducialData
{
public:
  /// \brief Fiducial ID
  std::string id;

  /// \brief Center point of the fiducial in the image
  ignition::math::Vector2i pt;
};


}
#endif
