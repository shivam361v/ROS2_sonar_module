#include "rclcpp/rclcpp.hpp"

#include <cassert>
#include <sys/stat.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // For TF2 conversions
// #include <tf2_ros/transform_listener.h>  // TF2 transform handling
// #include <geometry_msgs/msg/transform_stamped.hpp>  // Transformation messages

#include <sensor_msgs/msg/image.hpp>  // Updated for ROS 2
#include <sensor_msgs/msg/point_cloud2.hpp>  // Updated for ROS 2
//#include <sensor_msgs/sensor_msgs/point_cloud2_iterator.hpp>  // Updated for ROS 2

#include <cv_bridge/cv_bridge.h>  // Same as ROS 1

#include <pcl_ros/point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include <nps_uw_multibeam_sonar_converted/sonar_calculation_cuda.cuh>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <functional>
#include <nps_uw_multibeam_sonar_converted/base_interface.hpp>
#include <sdf/sdf.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/rendering/GpuLaser.hh>

#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <chrono>


namespace gazebo_ros
{

  GZ_REGISTER_SENSOR_PLUGIN(GazeboMultibeamSonar)

  GazeboMultibeamSonar::GazeboMultibeamSonar() : SensorPlugin(), width(0), height(0)
{
  this->point_cloud_connect_count_ = 0;
  this->sonar_image_connect_count_ = 0;

  // for csv write logs
  this->writeCounter = 0;
  this->writeNumber = 1;
}

GazeboMultibeamSonar::~GazeboMultibeamSonar() {
    // Reset connections and clear resources
    this->newLaserFrameConnection.reset();
    this->parentSensor.reset();
    this->laserCamera.reset();

    // Close CSV log file
    if (writeLog.is_open()) {
      writeLog.close();
    }
    
    rclcpp::shutdown();  // Shutdown ROS 2 when the plugin is destructed
}

void GazeboMultibeamSonar::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
  {
    // Convert the sensor to the appropriate type (e.g., GpuRaySensor)
    this->parentSensor = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(_sensor);
    this->laserCamera = this->parentSensor->LaserCamera();

    if (!this->parentSensor)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GazeboMultibeamSonar"), "Not attached to a GpuLaser sensor");
      return;
    }

    // Configuration parameters from SDF
    this->point_cloud_topic_name_ = _sdf->HasElement("pointCloudTopicName") ?
                                      _sdf->GetElement("pointCloudTopicName")->Get<std::string>() : "points";
    this->point_cloud_cutoff_ = _sdf->HasElement("pointCloudCutoff") ?
                                  _sdf->GetElement("pointCloudCutoff")->Get<double>() : 0.01;

    this->width = this->parentSensor->RangeCount();
    this->height = this->parentSensor->VerticalRangeCount();
    this->format = "R8G8B8";  // Default image format

    // Connect the laser frame callback
    this->newLaserFrameConnection = this->laserCamera->ConnectNewLaserFrame(
      std::bind(&GazeboMultibeamSonar::OnNewLaserFrame, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

    // Set sensor to active
    this->parentSensor->SetActive(true);

    // Additional properties
    this->sonar_image_raw_topic_name_ = _sdf->HasElement("sonarImageRawTopicName") ?
      _sdf->GetElement("sonarImageRawTopicName")->Get<std::string>() : "sonar_image_raw";

    this->sonar_image_topic_name_ = _sdf->HasElement("sonarImageTopicName") ?
      _sdf->GetElement("sonarImageTopicName")->Get<std::string>() : "sonar_image";

    // Sonar specific parameters
    this->verticalFOV = _sdf->HasElement("verticalFOV") ? _sdf->GetElement("verticalFOV")->Get<double>() : 10.0;
    this->sonarFreq = _sdf->HasElement("sonarFreq") ? _sdf->GetElement("sonarFreq")->Get<double>() : 900e3;
    this->bandwidth = _sdf->HasElement("bandwidth") ? _sdf->GetElement("bandwidth")->Get<double>() : 29.5e6;
    this->soundSpeed = _sdf->HasElement("soundSpeed") ? _sdf->GetElement("soundSpeed")->Get<double>() : 1500.0;
    this->maxDistance = _sdf->HasElement("maxDistance") ? _sdf->GetElement("maxDistance")->Get<double>() : 60.0;

    // Log sonar configuration
    RCLCPP_INFO(rclcpp::get_logger("GazeboMultibeamSonar"),
                "Sonar plugin loaded with settings:\n"
                "Max Distance: %.2f\n"
                "Distance resolution: %.2f\n"
                "Beams: %d\n"
                "Rays per beam: (%d, %d)",
                this->maxDistance,
                this->soundSpeed * (1.0 / (this->nFreq * delta_f)),
                this->nBeams,
                this->ray_nElevationRays, this->ray_nAzimuthRays);

    // Check for log writing configuration
    if (_sdf->HasElement("writeLog"))
    {
      this->writeLogFlag = _sdf->Get<bool>("writeLog");
      if (this->writeLogFlag)
      {
        if (_sdf->HasElement("writeFrameInterval"))
          this->writeInterval = _sdf->Get<int>("writeFrameInterval");
        else
          this->writeInterval = 10;

        RCLCPP_INFO(rclcpp::get_logger("GazeboMultibeamSonar"),
                    "Raw data logged every %d frames", this->writeInterval);
      }
    }

    // Sensor pre-calculation
    this->rand_image = cv::Mat(this->height, this->width, CV_32FC2);
    uint64 randN = static_cast<uint64>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.0f, 1.0f);

    // Hamming window
    this->window = new float[this->nFreq];
    float windowSum = 0;
    for (size_t f = 0; f < this->nFreq; f++)
    {
      this->window[f] = 0.54 - 0.46 * cos(2.0 * M_PI * (f + 1) / this->nFreq);
      windowSum += pow(this->window[f], 2.0);
    }
    for (size_t f = 0; f < this->nFreq; f++)
      this->window[f] = this->window[f] / sqrt(windowSum);

    // Prepare beam corrector
    this->beamCorrector = new float*[nBeams];
    for (int i = 0; i < nBeams; i++)
      this->beamCorrector[i] = new float[nBeams];

    // Connection setup for load
    this->load_connection_ = GazeboRosCameraUtils::OnLoad(
        boost::bind(&GazeboMultibeamSonar::Advertise, this));
    GazeboRosCameraUtils::Load(_sensor, _sdf);
  }

void GazeboMultibeamSonar::pointCloudSubThread()
{
    // The point cloud processing loop
    rclcpp::spin_some(this->get_node_base_interface());
    while (rclcpp::ok())
    {
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Simulate some work
    }
}

void GazeboMultibeamSonar::Advertise()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  // Publisher for point cloud
  this->point_cloud_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>(
      this->point_cloud_topic_name_, qos);

  // Publisher for normal image
  this->normal_image_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::Image>(
      this->point_cloud_topic_name_ + "_normal_image", qos);

  // Publisher for sonar raw data
  this->sonar_image_raw_pub_ = this->rosnode_->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
      this->sonar_image_raw_topic_name_, qos);

  // Publisher for sonar image
  this->sonar_image_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::Image>(
      this->sonar_image_topic_name_, qos);

  // Subscription to point cloud
  auto callback = std::bind(&NpsGazeboRosMultibeamSonarRay::UpdatePointCloud, this, std::placeholders::_1);
  this->point_cloud_sub_ = this->rosnode_->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->point_cloud_topic_name_, qos, callback);
}

void GazeboMultibeamSonar::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  this->parentSensor->SetActive(true);
}

void GazeboMultibeamSonar::PointCloudDisconnect()
{
  if (this->point_cloud_connect_count_ > 0)
    this->point_cloud_connect_count_--;

  if (this->point_cloud_connect_count_ == 0)
    this->parentSensor->SetActive(false);
}

void GazeboMultibeamSonar::SonarImageConnect()
{
  this->sonar_image_connect_count_++;
  this->parentSensor->SetActive(true);
}

void GazeboMultibeamSonar::SonarImageDisconnect()
{
  if (this->sonar_image_connect_count_ > 0)
    this->sonar_image_connect_count_--;

  if (this->sonar_image_connect_count_ == 0)
    this->parentSensor->SetActive(false);
}

void GazeboMultibeamSonar::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format)
{
  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (this->parentSensor->IsActive())
  {
    if (this->sonar_image_connect_count_ > 0 && !this->point_cloud_image_.empty())
    {
      this->ComputeSonarImage();
    }
  }
  else
  {
    if (this->sonar_image_connect_count_ > 0)
    {
      this->parentSensor->SetActive(true);
    }
  }
}

void GazeboMultibeamSonar::ComputeSonarImage()
{
  std::lock_guard<std::mutex> lock(this->lock_);

  if (this->point_cloud_image_.empty())
    return;

  cv::Mat depth_image = this->point_cloud_image_;
  cv::Mat normal_image = this->ComputeNormalImage(depth_image);
  double vFOV = this->parentSensor->VertFOV();
  double hFOV = this->parentSensor->HorzFOV();
  double vPixelSize = vFOV / (this->height - 1);
  double hPixelSize = hFOV / (this->width - 1);

  if (this->beamCorrectorSum == 0)
    ComputeCorrector();

  if (this->reflectivityImage.empty())
    this->reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

  auto start = std::chrono::high_resolution_clock::now();

  // Perform sonar calculations
  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
      depth_image, normal_image, rand_image,
      hPixelSize, vPixelSize, hFOV, vFOV,
      hPixelSize, verticalFOV / 180 * M_PI, hPixelSize,
      this->elevation_angles, vPixelSize * (raySkips + 1),
      this->soundSpeed, this->maxDistance, this->sourceLevel,
      this->nBeams, this->nRays, this->raySkips, this->sonarFreq,
      this->bandwidth, this->nFreq, this->reflectivityImage,
      this->attenuation, this->window, this->beamCorrector,
      this->beamCorrectorSum, this->debugFlag);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  if (debugFlag)
  {
    RCLCPP_INFO(this->rosnode_->get_logger(), "GPU Sonar Frame Calc Time: %ld µs", duration.count());
  }

  // Prepare sonar image message
  this->sonar_image_raw_msg_.header.frame_id = this->frame_name_;
  this->sonar_image_raw_msg_.header.stamp = this->rosnode_->now();

  marine_acoustic_msgs::msg::PingInfo ping_info_msg_;
  ping_info_msg_.frequency = this->sonarFreq;
  ping_info_msg_.sound_speed = this->soundSpeed;

  for (size_t beam = 0; beam < nBeams; beam++)
  {
    ping_info_msg_.rx_beamwidths.push_back(static_cast<float>(hFOV / floor(nBeams * 2.0 - 2.0) * 2.0));
    ping_info_msg_.tx_beamwidths.push_back(static_cast<float>(vFOV));
  }

  this->sonar_image_raw_msg_.ping_info = ping_info_msg_;

  for (size_t beam = 0; beam < nBeams; beam++)
  {
    geometry_msgs::msg::Vector3 beam_direction;
    beam_direction.x = cos(azimuth_angles[beam]);
    beam_direction.y = sin(azimuth_angles[beam]);
    beam_direction.z = 0.0;
    this->sonar_image_raw_msg_.beam_directions.push_back(beam_direction);
  }

  std::vector<float> ranges;
  for (size_t i = 0; i < P_Beams[0].size(); i++)
    ranges.push_back(rangeVector[i]);

  this->sonar_image_raw_msg_.ranges = ranges;

  // Fill sonar image data
  marine_acoustic_msgs::msg::SonarImageData sonar_image_data;
  sonar_image_data.is_bigendian = false;
  sonar_image_data.dtype = 0; // DTYPE_UINT8
  sonar_image_data.beam_count = nBeams;

  std::vector<uint8_t> intensities;
  int Intensity[nBeams][nFreq];

  for (size_t f = 0; f < nFreq; f++)
  {
    for (size_t beam = 0; beam < nBeams; beam++)
    {
      Intensity[beam][f] = static_cast<int>(this->sensorGain * abs(P_Beams[beam][f]));
      uint8_t counts = static_cast<uint8_t>(std::min(UCHAR_MAX, Intensity[beam][f]));
      intensities.push_back(counts);
    }
  }

  sonar_image_data.data = intensities;
  this->sonar_image_raw_msg_.image = sonar_image_data;
  this->sonar_image_raw_pub_->publish(this->sonar_image_raw_msg_);

  // Convert intensity data to visual sonar image
  cv_bridge::CvImage img_bridge;
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);
  cv::normalize(Intensity_image, Intensity_image, -255 + this->plotScaler / 10 * 255, 255, cv::NORM_MINMAX);
  cv::Mat Intensity_image_color;
  cv::applyColorMap(Intensity_image, Intensity_image_color, cv::COLORMAP_HOT);

  // Publish visual sonar image
  this->sonar_image_msg_.header.frame_id = this->frame_name_;
  this->sonar_image_msg_.header.stamp = this->rosnode_->now();
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header, sensor_msgs::image_encodings::BGR8, Intensity_image_color);
  img_bridge.toImageMsg(this->sonar_image_msg_);
  this->sonar_image_pub_->publish(this->sonar_image_msg_);

  // Publish normal image
  this->normal_image_msg_.header.frame_id = this->frame_name_;
  this->normal_image_msg_.header.stamp = this->rosnode_->now();
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  img_bridge = cv_bridge::CvImage(this->normal_image_msg_.header, sensor_msgs::image_encodings::RGB8, normal_image8);
  img_bridge.toImageMsg(this->normal_image_msg_);
  this->normal_image_pub_->publish(this->normal_image_msg_);
}

void GazeboMultibeamSonar::UpdatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(this->lock_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pcl_pointcloud);

  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();
  double hFOV = this->parentSensor->HorzFOV();

  // Calculate azimuth/elevation angles only if they haven't been set yet
  bool angles_calculation_flag = this->azimuth_angles.empty();

  for (int j = 0; j < this->nRays; j++)
  {
    if (angles_calculation_flag)
    {
      const double Diff = this->parentSensor->VerticalAngleMax().Radian()
                        - this->parentSensor->VerticalAngleMin().Radian();
      this->elevation_angles[j] = (j * Diff / (this->nRays - 1))
                                + this->parentSensor->VerticalAngleMin().Radian();
    }

    for (int i = 0; i < this->nBeams; i++, ++iter_image)
    {
      pcl::PointXYZI point = pcl_pointcloud->at(j, this->width - i - 1);
      this->point_cloud_image_.at<float>(j, i) = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      if (angles_calculation_flag && j == 0)
      {
        const double Diff = this->parentSensor->AngleMax().Radian()
                          - this->parentSensor->AngleMin().Radian();
        this->azimuth_angles.push_back((i * Diff / (this->nBeams - 1))
                          + this->parentSensor->AngleMin().Radian());
      }

      if (std::isnan(*iter_image))
        *iter_image = 100000.0;  // Assign large value to avoid processing issues
    }
  }

  // Publish point cloud if there are subscribers
  if (this->point_cloud_connect_count_ > 0)
  {
    this->point_cloud_pub_->publish(*msg);
  }
}

void GazeboMultibeamSonar::ComputeCorrector()
{
  double hFOV = this->parentSensor->HorzFOV();
  double hPixelSize = hFOV / (this->width - 1);
  
  // Reset the beamCorrectorSum
  this->beamCorrectorSum = 0.0;

  // Compute beam culling correction
  for (size_t beam = 0; beam < nBeams; beam++)
  {
    for (size_t beam_other = 0; beam_other < nBeams; beam_other++)
    {
      float azimuthBeamPattern = unnormalized_sinc(M_PI * 0.884 / hPixelSize
                                    * std::sin(this->azimuth_angles[beam] - this->azimuth_angles[beam_other]));
      
      this->beamCorrector[beam][beam_other] = std::abs(azimuthBeamPattern);
      this->beamCorrectorSum += std::pow(azimuthBeamPattern, 2);
    }
  }

  this->beamCorrectorSum = std::sqrt(this->beamCorrectorSum);

  // Debugging output if needed
  RCLCPP_INFO(this->rosnode_->get_logger(), "Beam corrector computed, sum: %f", this->beamCorrectorSum);
}

cv::Mat GazeboMultibeamSonar::ComputeNormalImage(cv::Mat& depth)


} // namespace gazebo_ros

// Register the plugin
PLUGINLIB_EXPORT_CLASS(gazebo_ros::P900SonarPlugin, gazebo_ros::BaseInterface)