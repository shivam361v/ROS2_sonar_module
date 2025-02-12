// Use ament_index_cpp to locate package share directories instead of ros/package.h
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cassert>
#include <sys/stat.h>

// In ROS 2, instead of tf/tf.h use tf2 and its geometry conversion header.
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS 2 image encodings header
#include <sensor_msgs/image_encodings.hpp>
// cv_bridge header in ROS 2 uses .hpp
#include <cv_bridge/cv_bridge/cv_bridge.h>

// Update marine_acoustic_msgs headers to ROS 2 message style
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>

// Update point cloud iterator header for ROS 2
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Custom CUDA sonar calculation header remains the same.
#include <nps_uw_multibeam_sonar_converted/sonar_calculation_cuda.cuh>

// OpenCV header (use new location)
#include <opencv2/core.hpp>

// Boost headers (these remain similar; note: ROS 2 prefers std::bind over boost::bind,
// but if you must keep boost you can include them as below)
// #include <boost/thread/thread.hpp>
// #include <boost/bind.hpp>

// Custom Gazebo plugin header (assumed to be updated separately)
#include <nps_uw_multibeam_sonar_converted/gazebo_multibeam_sonar_raster_based_header.hh>

// Gazebo Classic includes (using Gazebo Classic API)
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRosMultibeamSonar)

    // Constructor
NpsGazeboRosMultibeamSonar::NpsGazeboRosMultibeamSonar() :
  SensorPlugin(), width(0), height(0), depth(0)
{
  this->depth_image_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->point_cloud_connect_count_ = 0;
  this->sonar_image_connect_count_ = 0;
  // Use ROS 2 time for the last update. Note: This assumes that your simulation or node
  // time is being set appropriately.
  this->last_depth_image_camera_info_update_time_ = rclcpp::Time(0);

  // Frame counter for variational reflectivity
  this->maxDepth_before = 0.0;
  this->maxDepth_beforebefore = 0.0;
  this->maxDepth_prev = 0.0;

  // For CSV log writes
  this->writeCounter = 0;
  this->writeNumber = 1;
}

// Destructor
NpsGazeboRosMultibeamSonar::~NpsGazeboRosMultibeamSonar()
{
  this->newDepthFrameConnection_.reset();
  this->newImageFrameConnection_.reset();
  this->newRGBPointCloudConnection_.reset();

  this->parentSensor.reset();
  this->depthCamera.reset();

  // Close the CSV log stream if open.
  if (writeLog.is_open())
    writeLog.close();
}

  // Load the controller (ROS 2 version)
  void NpsGazeboRosMultibeamSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)


  {
    // Cast to a DepthCameraSensor
    this->parentSensor = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_parent);
    if (!this->parentSensor)
    {
      gzerr << "DepthCameraPlugin not attached to a depthCamera sensor\n";
      return;
    }

    

    this->depthCamera = this->parentSensor->DepthCamera();
    this->world = physics::get_world(parentSensor->WorldName());

    // Set image dimensions and format.
    this->width = this->depthCamera->ImageWidth();
    this->height = this->depthCamera->ImageHeight();
    this->depth = this->depthCamera->ImageDepth();
    this->format = this->depthCamera->ImageFormat();

    // Connect new depth and image frame callbacks using std::bind
    this->newDepthFrameConnection_ =
      this->depthCamera->ConnectNewDepthFrame(
          std::bind(&NpsGazeboRosMultibeamSonar::OnNewDepthFrame, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4,
                    std::placeholders::_5));

    this->newImageFrameConnection_ =
      this->depthCamera->ConnectNewImageFrame(
          std::bind(&NpsGazeboRosMultibeamSonar::OnNewImageFrame, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4,
                    std::placeholders::_5));

    this->parentSensor->SetActive(true);

    // Ensure ROS 2 is initialized (typically, the node_ member is created elsewhere or here)
    if (!rclcpp::ok())
    {
      RCLCPP_FATAL(rclcpp::get_logger("NpsGazeboRosMultibeamSonar"),
                   "ROS 2 is not initialized, unable to load plugin. "
                   "Make sure to load the gazebo_ros system plugin.");
      return;
    }

    // Copy sensor parameters to internal variables.
    this->parentSensor = this->parentSensor;
    this->width = this->width;
    this->height = this->height;
    this->depth = this->depth;
    this->format = this->format;
    this->depthCamera = this->depthCamera;

    // Set topic names based on SDF parameters (with defaults)
    if (!_sdf->HasElement("imageTopicName"))
      this->depth_image_topic_name_ = "ir/image_raw";
        if (!_sdf->HasElement("cameraInfoTopicName"))
      this->depth_image_camera_info_topic_name_ = "ir/camera_info";
    else
      this->depth_image_camera_info_topic_name_ = _sdf->GetElement("cameraInfoTopicName")->Get<std::string>();

    if (!_sdf->HasElement("depthImageTopicName"))
      this->depth_image_topic_name_ = "depth/image_raw";
    else
      this->depth_image_topic_name_ = _sdf->GetElement("depthImageTopicName")->Get<std::string>();

    if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
      this->depth_image_camera_info_topic_name_ = "depth/camera_info";
    else
      this->depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();

    if (!_sdf->HasElement("pointCloudTopicName"))
      this->point_cloud_topic_name_ = "points";
    else
      this->point_cloud_topic_name_ = _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

    if (!_sdf->HasElement("pointCloudCutoff"))
      this->point_cloud_cutoff_ = 1;
    else
      this->point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->Get<double>();

    // Sonar topics
    if (!_sdf->HasElement("sonarImageRawTopicName"))
      this->sonar_image_raw_topic_name_ = "sonar_image_raw";
    else
      this->sonar_image_raw_topic_name_ = _sdf->GetElement("sonarImageRawTopicName")->Get<std::string>();

    if (!_sdf->HasElement("sonarImageTopicName"))
      this->sonar_image_topic_name_ = "sonar_image";
    else
      this->sonar_image_topic_name_ = _sdf->GetElement("sonarImageTopicName")->Get<std::string>();

    // Read sonar properties from the SDF file
    if (!_sdf->HasElement("verticalFOV"))
      this->verticalFOV = 30;
    else
      this->verticalFOV = _sdf->GetElement("verticalFOV")->Get<double>();

    if (!_sdf->HasElement("sonarFreq"))
      this->sonarFreq = 900e3;
    else
      this->sonarFreq = _sdf->GetElement("sonarFreq")->Get<double>();

    if (!_sdf->HasElement("bandwidth"))
      this->bandwidth = 29.5e6;
    else
      this->bandwidth = _sdf->GetElement("bandwidth")->Get<double>();

    if (!_sdf->HasElement("soundSpeed"))
      this->soundSpeed = 1500;
    else
      this->soundSpeed = _sdf->GetElement("soundSpeed")->Get<double>();

    if (!_sdf->HasElement("maxDistance"))
      this->maxDistance = 60;
    else
      this->maxDistance = _sdf->GetElement("maxDistance")->Get<double>();

    if (!_sdf->HasElement("sourceLevel"))
      this->sourceLevel = 220;
    else
      this->sourceLevel = _sdf->GetElement("sourceLevel")->Get<double>();

    if (!_sdf->HasElement("constantReflectivity"))
      this->constMu = true;
    else
      this->constMu = _sdf->GetElement("constantReflectivity")->Get<bool>();

    if (!_sdf->HasElement("artificialVehicleVibration"))
      this->artificialVehicleVibration = false;
    else
      this->artificialVehicleVibration = _sdf->GetElement("artificialVehicleVibration")->Get<bool>();

    if (!_sdf->HasElement("customSDFTagReflectivity"))
      this->customTag = false;
    else
      this->customTag = _sdf->GetElement("customSDFTagReflectivity")->Get<bool>();

    if (!_sdf->HasElement("raySkips"))
      this->raySkips = 10;
    else
      this->raySkips = _sdf->GetElement("raySkips")->Get<int>();

    if (!_sdf->HasElement("plotScaler"))
      this->plotScaler = 10;
    else
      this->plotScaler = _sdf->GetElement("plotScaler")->Get<float>();

    if (!_sdf->HasElement("sensorGain"))
      this->sensorGain = 0.02;
    else
      this->sensorGain = _sdf->GetElement("sensorGain")->Get<float>();

    if (this->raySkips == 0) this->raySkips = 1;

    // --- Variational Reflectivity --- //
    if (!this->constMu)
    {
      if (!this->customTag)
      {
        if (!_sdf->HasElement("reflectivityDatabaseFile"))
          this->reflectivityDatabaseFileName = "variationalReflectivityDatabase.csv";
        else
          this->reflectivityDatabaseFileName = _sdf->GetElement("reflectivityDatabaseFile")->Get<std::string>();
      }
      else
      {
        if (!_sdf->HasElement("customSDFTagDatabaseFile"))
          this->customTagDatabaseFileName = "customSDFTagDatabase.csv";
        else
          this->customTagDatabaseFileName = _sdf->GetElement("customSDFTagDatabaseFile")->Get<std::string>();
      }
    }
    this->mu = 1e-3;  // default constant mu

    // Use ament_index_cpp to get the package share directory.
    std::string packageShareDir = ament_index_cpp::get_package_share_directory("nps_uw_multibeam_sonar");
    this->reflectivityDatabaseFilePath = packageShareDir + "/worlds/" + this->reflectivityDatabaseFileName;
    this->customTagDatabaseFilePath = packageShareDir + "/worlds/" + this->customTagDatabaseFileName;

    // Read CSV file for reflectivity data.
    std::ifstream csvFile;
    std::string line;
    if (!this->customTag)
      csvFile.open(this->reflectivityDatabaseFilePath);
    else
      csvFile.open(this->customTagDatabaseFilePath);
    // Skip the first three lines.
    std::getline(csvFile, line);
    std::getline(csvFile, line);
    std::getline(csvFile, line);
    while (std::getline(csvFile, line))
    {
      if (line.empty())
        continue;
      std::istringstream iss(line);
      std::string token;
      std::vector<std::string> row;
      while (std::getline(iss, token, ','))
      {
        row.push_back(token);
      }
      this->objectNames.push_back(row[0]);
      this->reflectivities.push_back(std::stof(row[1]));
    }

    // Read coefficients for biofouling and roughness if customTag is set.
    if (this->customTag)
    {
      for (size_t k = 0; k < objectNames.size(); k++)
      {
        if (objectNames[k] == "biofouling_rating")
          this->biofouling_rating_coeff = reflectivities[k];
        if (objectNames[k] == "roughness")
          this->roughness_coeff = reflectivities[k];
      }
    }

    // Get the scene pointer from the depth camera.
    if (this->depthCamera)
      this->scene = this->depthCamera->GetScene();
    if (!this->depthCamera || !this->scene)
    {
      gzerr << "SonarDummy failed to load. Camera and/or Scene not found" << std::endl;
    }

    // Load fiducials from SDF or set detectAll if none provided.
    if (_sdf->HasElement("fiducial"))
    {
      sdf::ElementPtr elem = _sdf->GetElement("fiducial");
      while (elem)
      {
        this->fiducials.insert(elem->Get<std::string>());
        elem = elem->GetNextElement("fiducial");
      }
    }
    else
    {
      gzmsg << "No fiducials specified. All models will be tracked." << std::endl;
      this->detectAll = true;
    }

    // Transmission path properties
    this->absorption = 0.0354;  // [dB/m]\n    this->attenuation = this->absorption * std::log(10) / 20.0;

    // Range vector calculation
    const float max_T = static_cast<float>(this->maxDistance * 2.0 / this->soundSpeed);
    float delta_f = 1.0f / max_T;
    const float delta_t = 1.0f / static_cast<float>(this->bandwidth);
    this->nFreq = static_cast<int>(std::ceil(this->bandwidth / delta_f));
    delta_f = static_cast<float>(this->bandwidth) / this->nFreq;
    const int nTime = this->nFreq;
    this->rangeVector = new float[nTime];
    for (int i = 0; i < nTime; i++)
    {
      this->rangeVector[i] = delta_t * i * this->soundSpeed / 2.0;
    }

    // FOV, beams, and rays settings: width equals number of beams, height equals number of rays.
    this->nBeams = this->width;
    this->nRays = this->height;
    this->ray_nElevationRays = this->height;
    this->ray_nAzimuthRays = 1;
    this->elevation_angles = new float[this->nRays];

    // Print sonar settings using ROS 2 logging.
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "==================================================");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "============   SONAR PLUGIN LOADED   =============");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "==================================================");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "============      RASTER VERSION     =============");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "==================================================");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Maximum view range  [m] = " << this->maxDistance);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Distance resolution [m] = " <<
                        this->soundSpeed * (1.0 / (this->nFreq * delta_f)));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "# of Beams = " << this->nBeams);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "# of Rays / Beam (Elevation, Azimuth) = ("
                        << ray_nElevationRays << ", " << ray_nAzimuthRays << ")");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Calculation skips (Elevation) = " << this->raySkips);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "# of Time data / Beam = " << this->nFreq);
    if (!this->constMu)
    {
      if (this->customTag)
        RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Reflectivity method : Variational (based on custom SDF tag)");
      else
        RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Reflectivity method : Variational (based on model name)");
    }
    else
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Reflectivity method : Constant");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "==================================================");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "");

    // Get the write log flag
    if (!_sdf->HasElement("writeLog"))
      this->writeLogFlag = false;
    else
    {
      this->writeLogFlag = _sdf->Get<bool>("writeLog");
      if (this->writeLogFlag)
      {
        if (_sdf->HasElement("writeFrameInterval"))
          this->writeInterval = _sdf->Get<int>("writeFrameInterval");
        else
          this->writeInterval = 10;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "Raw data at /tmp/SonarRawData_{numbers}.csv every " << this->writeInterval << " frames");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("sonar_plugin"), "");

        struct stat buffer;
        std::string logfilename("/tmp/SonarRawData_000001.csv");
        if (stat(logfilename.c_str(), &buffer) == 0)
          system("rm /tmp/SonarRawData*.csv");
      }
    }

    // Debug flag for computation time display.
    if (!_sdf->HasElement("debugFlag"))
      this->debugFlag = false;
    else
      this->debugFlag = _sdf->GetElement("debugFlag")->Get<bool>();

    // --- Pre-calculations for sonar --- //
    this->rand_image = cv::Mat(this->height, this->width, CV_32FC2);
    uint64_t randN = static_cast<uint64_t>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.0f);

    // Hamming window calculation
    this->window = new float[this->nFreq];
    float windowSum = 0;
    for (size_t f = 0; f < this->nFreq; f++)
    {
      this->window[f] = 0.54f - 0.46f * std::cos(2.0 * M_PI * (f + 1) / this->nFreq);
      windowSum += std::pow(this->window[f], 2.0f);
    }
    for (size_t f = 0; f < this->nFreq; f++)
      this->window[f] = this->window[f] / std::sqrt(windowSum);

    // Sonar corrector preallocation
    this->beamCorrector = new float*[nBeams];
    for (int i = 0; i < nBeams; i++)
      this->beamCorrector[i] = new float[nBeams];
    this->beamCorrectorSum = 0.0f;
  }

// PopulateFiducials() – no ROS-specific changes required.
void NpsGazeboRosMultibeamSonar::PopulateFiducials()
{
  this->fiducials.clear();

  // Check all models for inclusion in the frustum.
  rendering::VisualPtr worldVis = this->scene->WorldVisual();
  for (unsigned int i = 0; i < worldVis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = worldVis->GetChild(i);
    if (childVis->GetType() == rendering::Visual::VT_MODEL)
      this->fiducials.insert(childVis->Name());
  }
}

// Advertise() – ROS 2 version using rclcpp publishers.
void NpsGazeboRosMultibeamSonar::Advertise()

{
  // Create a publisher for the depth image.
  this->depth_image_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::Image>(this->depth_image_topic_name_, 10);

  // Create a publisher for the depth camera info.
  this->depth_image_camera_info_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::CameraInfo>(this->depth_image_camera_info_topic_name_, 10);

  // Create a publisher for the normal image.
  this->normal_image_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::Image>(this->depth_image_topic_name_ + "_normals", 10);

  // Create a publisher for the point cloud.
  this->point_cloud_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>(this->point_cloud_topic_name_, 10);

  // Create a publisher for the sonar image raw message.
  this->sonar_image_raw_pub_ = this->rosnode_->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(this->sonar_image_raw_topic_name_, 10);

  // Create a publisher for the sonar image.
  this->sonar_image_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::Image>(this->sonar_image_topic_name_, 10);
}

  //=== Connection callback methods ===//

  /// brief Called when a new subscription for the depth image is made.
  void DepthImageConnect()
  {
    ++this->depth_image_connect_count_;
    // Activate the sensor if it was not active.
    this->parentSensor->SetActive(true);
    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "DepthImage connected. Count: %d", this->depth_image_connect_count_);
  }

  /// \brief Called when a subscription for the depth image is removed.
  void DepthImageDisconnect()
  {
    if (this->depth_image_connect_count_ > 0)
      --this->depth_image_connect_count_;

    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "DepthImage disconnected. Count: %d", this->depth_image_connect_count_);

    // Optionally, if no connections remain, deactivate the sensor.
    if (this->depth_image_connect_count_ == 0 && this->point_cloud_connect_count_ == 0)
      this->parentSensor->SetActive(false);
  }

  /// \brief Called when a new subscription for the normal image is made.
  void NormalImageConnect()
  {
    ++this->normal_image_connect_count_;
    this->parentSensor->SetActive(true);
    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "NormalImage connected. Count: %d", this->normal_image_connect_count_);
  }

  /// \brief Called when a subscription for the normal image is removed.
  void NormalImageDisconnect()
  {
    if (this->normal_image_connect_count_ > 0)
      --this->normal_image_connect_count_;

    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "NormalImage disconnected. Count: %d", this->normal_image_connect_count_);

    if (this->normal_image_connect_count_ == 0 && this->point_cloud_connect_count_ == 0)
      this->parentSensor->SetActive(false);
  }

  /// \brief Called when a new subscription for depth info is made.
  void DepthInfoConnect()
  {
    ++this->depth_info_connect_count_;
    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "DepthInfo connected. Count: %d", this->depth_info_connect_count_);
  }

  /// \brief Called when a subscription for depth info is removed.
  void DepthInfoDisconnect()
  {
    if (this->depth_info_connect_count_ > 0)
      --this->depth_info_connect_count_;
    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "DepthInfo disconnected. Count: %d", this->depth_info_connect_count_);
  }

  /// \brief Called when a new subscription for the point cloud is made.
  void PointCloudConnect()
  {
    ++this->point_cloud_connect_count_;
    // Optionally, if you maintain a separate image connect count, update it here.
    // For example: ++(*this->image_connect_count_);
    this->parentSensor->SetActive(true);
    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "PointCloud connected. Count: %d", this->point_cloud_connect_count_);
  }

  /// \brief Called when a subscription for the point cloud is removed.
  void PointCloudDisconnect()
  {
    if (this->point_cloud_connect_count_ > 0)
      --this->point_cloud_connect_count_;

    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "PointCloud disconnected. Count: %d", this->point_cloud_connect_count_);

    if (this->point_cloud_connect_count_ == 0)
      this->parentSensor->SetActive(false);
  }


// Called when Gazebo provides a new depth frame (texture)
void NpsGazeboRosMultibeamSonar::OnNewDepthFrame(const float *_image,
                                                 unsigned int _width,
                                                 unsigned int _height,
                                                 unsigned int _depth,
                                                 const std::string &_format)
{
  // Check if the plugin is properly initialized and if the image dimensions are valid.
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
    return;

  // Record the time stamp from the parent sensor
  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (this->parentSensor->IsActive())
  {
    // If there are no subscribers for depth image, point cloud, or the other image topic, deactivate.
    if (this->depth_image_connect_count_ <= 0 &&
        this->point_cloud_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
      RCLCPP_DEBUG(this->ros_node_->get_logger(),
                   "Deactivating sensor since no subscribers are present.");
    }
    else
    {
      // Process the depth frame by computing a point cloud.
      this->ComputePointCloud(_image);

      // Additionally, if there are subscribers for the depth image, compute the sonar image.
      if (this->depth_image_connect_count_ > 0)
      {
        this->ComputeSonarImage(_image);
      }
    }
  }
  else
  {
    // If the sensor is not active but there are subscribers (for point cloud or depth image),
    // then activate the sensor.
    if (this->depth_image_connect_count_ <= 0 || this->point_cloud_connect_count_ > 0)
    {
      this->parentSensor->SetActive(true);
      RCLCPP_DEBUG(this->ros_node_->get_logger(),
                   "Activating sensor based on new depth frame.");
    }
  }
}


// Called when Gazebo provides a new image frame
void NpsGazeboRosMultibeamSonar::OnNewImageFrame(const unsigned char *_image,
                                                 unsigned int _width,
                                                 unsigned int _height,
                                                 unsigned int _depth,
                                                 const std::string &_format)
{
  // Check if the plugin is properly initialized and if the image dimensions are valid.
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
    return;

  // Update the sensor timestamp using the parent sensor's last measurement time.
  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  // If the sensor is inactive and there are subscribers for the image topic, activate it.
  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
    {
      // Activate the sensor so it has a chance to run one frame after activation.
      this->parentSensor->SetActive(true);
      RCLCPP_DEBUG(this->ros_node_->get_logger(),
                   "Activating sensor due to new image frame with active subscriptions.");
    }
  }
  else
  {
    // When the sensor is active and there are subscribers, process the image frame.
    if ((*this->image_connect_count_) > 0)
    {
      this->PutCameraData(_image);
      RCLCPP_DEBUG(this->ros_node_->get_logger(),
                   "Processed new image frame.");
    }
  }

  // ----- Reflectivity Calculation (if applicable) ----- //
  // Calculate only if the maxDepth from depth camera is changed and stabilized.
  double minVal;
  cv::minMaxLoc(this->point_cloud_image_, &minVal, &this->maxDepth);
  if (this->maxDepth == this->maxDepth_before &&
      this->maxDepth == this->maxDepth_beforebefore &&
      !this->calculateReflectivity &&
      this->maxDepth != this->maxDepth_prev)
  {
    this->calculateReflectivity = true;
    this->maxDepth_prev = this->maxDepth;

    // Regenerate random image using OpenCV's RNG
    uint64_t randN = static_cast<uint64_t>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);

    RCLCPP_DEBUG(this->ros_node_->get_logger(),
                 "Regenerating random image for reflectivity computation.");
  }
  else
  {
    this->calculateReflectivity = false;
  }

  // Shift historical maxDepth values for subsequent frames.
  this->maxDepth_beforebefore = this->maxDepth_before;
  this->maxDepth_before = this->maxDepth;

  // ----- Variational Reflectivity Calculation ----- //
  if (!this->constMu)
  {
    if (this->calculateReflectivity)
    {
      // Create an OpenCV image for reflectivity, initialized with a constant value (mu)
      cv::Mat reflectivity_image = cv::Mat(_width, _height, CV_32FC1, cv::Scalar(this->mu));

      // Ensure the selection buffer is created if needed.
      if (!this->selectionBuffer)
      {
        std::string cameraName = this->camera_->OgreCamera()->getName();
        this->selectionBuffer.reset(
            new rendering::SelectionBuffer(
                cameraName,
                this->scene->OgreSceneManager(),
                this->camera_->RenderTexture()->getBuffer()->getRenderTarget()));
      }

      // Optionally detect fiducials if enabled.
      if (this->detectAll)
        this->PopulateFiducials();

      // Process each fiducial to compute reflectivity.
      for (const auto &f : this->fiducials)
      {
        // Get the visual associated with the fiducial; skip if not found.
        rendering::VisualPtr vis = this->scene->GetVisual(f);
        if (!vis)
          continue;

        // Skip if the fiducial is not visible in the current camera view.
        if (!this->depthCamera->IsVisible(vis))
          continue;

        RCLCPP_INFO(this->ros_node_->get_logger(),
                    "Calculating Reflectivity of captured objects using custom SDF Tags. This may take some time on the first frame.");

        // Loop over every pixel (with ray skipping for efficiency)
        for (int i = 0; i < reflectivity_image.rows; i++)
        {
          for (int j = 0; j < reflectivity_image.cols; j += raySkips)
          {
            // Define the target pixel.
            ignition::math::Vector2i pt(i, j);

            // Use the selection buffer to check if the visual is occluded.
            Ogre::Entity *entity = this->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

            rendering::VisualPtr result;
            if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
            {
              try
              {
                result = this->scene->GetVisual(
                    Ogre::any_cast<std::string>(
                        entity->getUserObjectBindings().getUserAny()));
              }
              catch(Ogre::Exception &_e)
              {
                gzerr << "Ogre Exception: " << _e.getFullDescription() << "\n";
                continue;
              }
            }

            // If the result visual corresponds to the fiducial's visual, assign reflectivity.
            if (result && result->GetRootVisual() == vis)
            {
              // For non-custom tags, check against predefined object names.
              if (!this->customTag)
              {
                for (size_t k = 0; k < objectNames.size(); ++k)
                {
                  if (vis->Name() == objectNames[k])
                  {
                    reflectivity_image.at<float>(j, i) = reflectivities[k];
                    break;
                  }
                }
              }
              else
              {
                // Read custom SDF tags for surface properties.
                sdf::ElementPtr modelElt = this->world->BaseByName(vis->Name())->GetSDF();

                int biofoulingRating = 0; // Rating between [0, 100]
                if (modelElt->HasElement("surface_props:biofouling_rating"))
                  biofoulingRating = modelElt->Get<int>("surface_props:biofouling_rating");

                double roughness = 0.0; // Roughness between [0.0, 1.0]
                if (modelElt->HasElement("surface_props:roughness"))
                  roughness = modelElt->Get<double>("surface_props:roughness");

                std::string material = "default";
                if (modelElt->HasElement("surface_props:material"))
                  material = modelElt->Get<std::string>("surface_props:material");

                for (size_t k = 0; k < objectNames.size(); ++k)
                {
                  if (material == objectNames[k])
                  {
                    reflectivity_image.at<float>(j, i) =
                      reflectivities[k] * (1.0 / (roughness + 1)) / this->roughness_coeff *
                      (1.0 / (biofoulingRating + 1)) / this->biofouling_rating_coeff;
                    break;
                  }
                }
              }
              // Optionally, you can store fiducial data if needed.
              // FiducialData fd;
              // fd.id = vis->Name();
              // fd.pt = pt;
              // results.push_back(fd);
            }
          }
        }  // End of pixel loops
      }  // End of fiducial processing

      // Save or publish the computed reflectivity image as needed.
      this->reflectivityImage = reflectivity_image;
      RCLCPP_DEBUG(this->ros_node_->get_logger(),
                   "Reflectivity image computed and stored.");
    }  // End if (calculateReflectivity)
  }  // End if (!constMu)
}



void NpsGazeboRosMultibeamSonar::ComputeSonarImage(const float *_src)
{
  // Use a lock guard for RAII style locking
  std::lock_guard<std::mutex> lock(this->lock_);

  // Use the point cloud image as the depth image
  cv::Mat depth_image = this->point_cloud_image_;

  // Compute a normal image from the depth image (user-defined function)
  cv::Mat normal_image = this->ComputeNormalImage(depth_image);

  // Get vertical and horizontal FOV (assumes DepthCamera() is valid)
  double vFOV = this->parentSensor->DepthCamera()->VFOV().Radian();
  double hFOV = this->parentSensor->DepthCamera()->HFOV().Radian();

  // Compute the pixel sizes
  double vPixelSize = vFOV / this->height;
  double hPixelSize = hFOV / this->width;

  // Compute beam corrector if needed
  if (this->beamCorrectorSum == 0)
    ComputeCorrector();

  // If no reflectivity image exists yet, initialize it with the default value mu
  if (this->reflectivityImage.rows == 0)
    this->reflectivityImage = cv::Mat(this->width, this->height, CV_32FC1, cv::Scalar(this->mu));

  // Regenerate random image if artificial vehicle vibration flag is on
  if (this->artificialVehicleVibration)
  {
    uint64_t randN = static_cast<uint64_t>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);
  }

  // Start time measurement
  auto start = std::chrono::high_resolution_clock::now();

  // ------------------------------------------------//
  // --------      Sonar calculations       -------- //
  // ------------------------------------------------//
  // Call the sonar calculation wrapper (user-defined function)
  // Note: The function signature is assumed to match the following parameters.
  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
                          depth_image,           // cv::Mat& depth_image
                          normal_image,          // cv::Mat& normal_image
                          this->rand_image,      // cv::Mat& rand_image
                          hPixelSize,            // horizontal pixel size
                          vPixelSize,            // vertical pixel size
                          hFOV,                  // horizontal FOV
                          vFOV,                  // vertical FOV
                          hPixelSize,            // beam azimuth angle width
                          verticalFOV / 180 * M_PI,  // beam elevation angle width
                          hPixelSize,            // ray azimuth angle width
                          this->elevation_angles,// ray elevation angles (vector)
                          vPixelSize * (raySkips + 1),  // ray elevation angle width (adjusted)
                          this->soundSpeed,      // sound speed
                          this->maxDistance,     // max distance
                          this->sourceLevel,     // source level
                          this->nBeams,          // number of beams
                          this->nRays,           // number of rays
                          this->raySkips,        // ray skips
                          this->sonarFreq,       // sonar frequency
                          this->bandwidth,       // bandwidth
                          this->nFreq,           // number of frequencies
                          this->reflectivityImage, // reflectivity image
                          this->attenuation,     // attenuation
                          this->window,          // window
                          this->beamCorrector,   // beam corrector
                          this->beamCorrectorSum,// beam corrector sum
                          this->debugFlag);      // debug flag

  // End time measurement
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  if (this->debugFlag)
  {
    RCLCPP_INFO(this->ros_node_->get_logger(),
                "GPU Sonar Frame Calc Time: %ld / 100 [s]", duration.count() / 10000);
  }

  // Optionally: add Gaussian noise here if needed
  // double whiteNoise = ignition::math::Rand::DblNormal(0.0, 0.7);

  // CSV logging if enabled
  if (this->writeLogFlag)
  {
    this->writeCounter++;
    if (this->writeCounter == 1 || (this->writeCounter % this->writeInterval == 0))
    {
      double time_sec = this->parentSensor->LastMeasurementTime().Double();
      std::stringstream filename;
      filename << "/tmp/SonarRawData_" << std::setw(6) << std::setfill('0')
               << this->writeNumber << ".csv";
      std::ofstream writeLog;
      writeLog.open(filename.str().c_str(), std::ios_base::app);
      filename.str("");
      writeLog << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n";
      writeLog << "# First column is range vector\n";
      writeLog << "#  nBeams : " << this->nBeams << "\n";
      writeLog << "# Simulation time : " << time_sec << "\n";
      for (size_t i = 0; i < P_Beams[0].size(); i++)
      {
        // Write the range vector in the first column
        writeLog << this->rangeVector[i];
        for (size_t b = 0; b < this->nBeams; b++)
        {
          if (P_Beams[b][i].imag() > 0)
            writeLog << "," << P_Beams[b][i].real()
                     << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog << "," << P_Beams[b][i].real()
                     << P_Beams[b][i].imag() << "i";
        }
        writeLog << "\n";
      }
      writeLog.close();
      this->writeNumber++;
    }
  }

  // Prepare the sonar image raw message (custom message type)
  this->sonar_image_raw_msg_.header.frame_id = this->frame_name_;
  this->sonar_image_raw_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->sonar_image_raw_msg_.header.stamp.nanosec = this->depth_sensor_update_time_.nsec;  // ROS 2 uses "nanosec"

  // Fill in ping info (assuming marine_acoustic_msgs::msg::PingInfo)
  marine_acoustic_msgs::msg::PingInfo ping_info_msg;
  ping_info_msg.frequency = this->sonarFreq;
  ping_info_msg.sound_speed = this->soundSpeed;
  std::vector<float> azimuth_angles;
  double fl = static_cast<double>(this->width) / (2.0 * tan(hFOV / 2.0));
  for (size_t beam = 0; beam < this->nBeams; beam++)
  {
    ping_info_msg.rx_beamwidths.push_back(static_cast<float>(
      fabs(atan2(static_cast<double>(beam) - 1.0 - 0.0 * static_cast<double>(this->width), fl)
      - atan2(static_cast<double>(beam), fl))));
    ping_info_msg.tx_beamwidths.push_back(static_cast<float>(vFOV));
    azimuth_angles.push_back(static_cast<float>(atan2(static_cast<double>(beam) - 0.5 * static_cast<double>(this->width), fl)));
  }
  this->sonar_image_raw_msg_.ping_info = ping_info_msg;

  // Compute beam directions
  std::vector<geometry_msgs::msg::Vector3> beam_directions_stack;
  for (size_t beam = 0; beam < this->nBeams; beam++)
  {
    geometry_msgs::msg::Vector3 beam_direction;
    beam_direction.x = cos(azimuth_angles[beam]);
    beam_direction.y = sin(azimuth_angles[beam]);
    beam_direction.z = 0.0;
    beam_directions_stack.push_back(beam_direction);
  }
  this->sonar_image_raw_msg_.beam_directions = beam_directions_stack;

  // Save range vector
  std::vector<float> ranges;
  for (size_t i = 0; i < P_Beams[0].size(); i++)
    ranges.push_back(this->rangeVector[i]);
  this->sonar_image_raw_msg_.ranges = ranges;

  // Prepare sonar image data (assuming marine_acoustic_msgs::msg::SonarImageData)
  marine_acoustic_msgs::msg::SonarImageData sonar_image_data;
  sonar_image_data.is_bigendian = false;
  sonar_image_data.dtype = 0;  // e.g. DTYPE_UINT8
  sonar_image_data.beam_count = this->nBeams;

  std::vector<uchar> intensities;
  int Intensity[this->nBeams][this->nFreq];
  for (size_t f = 0; f < this->nFreq; f++)
  {
    for (size_t beam = 0; beam < this->nBeams; beam++)
    {
      // Serialize beams in reverse order to flip the data left to right
      const size_t beam_idx = this->nBeams - beam - 1;
      Intensity[beam_idx][f] = static_cast<int>(this->sensorGain * fabs(P_Beams[beam_idx][f]));
      uchar counts = static_cast<uchar>(std::min<int>(std::numeric_limits<uchar>::max(), Intensity[beam_idx][f]));
      intensities.push_back(counts);
    }
  }
  sonar_image_data.data = intensities;
  this->sonar_image_raw_msg_.image = sonar_image_data;

  // Publish the sonar raw message
  this->sonar_image_raw_pub_->publish(this->sonar_image_raw_msg_);

  // ----- Visual Sonar Image for rqt Plot -----
  cv_bridge::CvImage img_bridge;

  // Generate an image of type 8UC1 (dimensions: nBeams x nFreq)
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(this->nBeams, this->nFreq), CV_8UC1);

  const float rangeMax = this->maxDistance;
  const float rangeRes = ranges[1] - ranges[0];
  const int nEffectiveRanges = static_cast<int>(ceil(rangeMax / rangeRes));
  const unsigned int radius = Intensity_image.size().height;
  const cv::Point origin(Intensity_image.size().width / 2, Intensity_image.size().height);
  const float binThickness = 2 * ceil(radius / static_cast<float>(nEffectiveRanges));

  // Structure to hold bearing information
  struct BearingEntry
  {
    float begin, center, end;
    BearingEntry(float b, float c, float e)
      : begin(b), center(c), end(e) {}
  };

  std::vector<BearingEntry> angles;
  angles.reserve(this->nBeams);

  for (int b = 0; b < static_cast<int>(this->nBeams); ++b)
  {
    const float center = azimuth_angles[b];
    float begin = 0.0f, end = 0.0f;
    if (b == 0)
    {
      end = (azimuth_angles[b + 1] + center) / 2.0f;
      begin = 2 * center - end;
    }
    else if (b == static_cast<int>(this->nBeams) - 1)
    {
      begin = angles[b - 1].end;
      end = 2 * center - begin;
    }
    else
    {
      begin = angles[b - 1].end;
      end = (azimuth_angles[b + 1] + center) / 2.0f;
    }
    angles.push_back(BearingEntry(begin, center, end));
  }

  const float ThetaShift = 1.5 * M_PI;
  for (size_t r = 0; r < ranges.size(); ++r)
  {
    if (ranges[r] > rangeMax)
      continue;
    for (int b = 0; b < static_cast<int>(this->nBeams); ++b)
    {
      const float range = ranges[r];
      // Calculate intensity from the sonar beams (use logarithmic scaling)
      int intensity = static_cast<int>(floor(10.0 * log(fabs(P_Beams[this->nBeams - 1 - b][r]))));
      const float begin = angles[b].begin + ThetaShift;
      const float end = angles[b].end + ThetaShift;
      const float rad = static_cast<float>(radius) * range / rangeMax;
      // Draw an ellipse for this beam segment on the image
      cv::ellipse(Intensity_image, origin, cv::Size(rad, rad), 0,
                  begin * 180 / M_PI, end * 180 / M_PI,
                  intensity, binThickness);
    }
  }

  // Normalize and apply a colormap for visualization
  cv::normalize(Intensity_image, Intensity_image, -255 + this->plotScaler / 10 * 255, 255, cv::NORM_MINMAX);
  cv::Mat Intensity_image_color;
  cv::applyColorMap(Intensity_image, Intensity_image_color, cv::COLORMAP_HOT);

  // Prepare and publish the sonar image message (sensor_msgs::msg::Image)
  this->sonar_image_msg_.header.frame_id = this->frame_name_;
  this->sonar_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->sonar_image_msg_.header.stamp.nanosec = this->depth_sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header,
                                  sensor_msgs::image_encodings::BGR8,
                                  Intensity_image_color);
  this->sonar_image_msg_ = *img_bridge.toImageMsg();
  this->sonar_image_pub_->publish(this->sonar_image_msg_);

  // ----- Publish the depth image -----
  this->depth_image_msg_.header.frame_id = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nanosec = this->depth_sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->depth_image_msg_.header,
                                  sensor_msgs::image_encodings::TYPE_32FC1,
                                  depth_image);
  this->depth_image_msg_ = *img_bridge.toImageMsg();
  this->depth_image_pub_->publish(this->depth_image_msg_);

  // ----- Publish the normal image -----
  this->normal_image_msg_.header.frame_id = this->frame_name_;
  this->normal_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->normal_image_msg_.header.stamp.nanosec = this->depth_sensor_update_time_.nsec;
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  img_bridge = cv_bridge::CvImage(this->normal_image_msg_.header,
                                  sensor_msgs::image_encodings::RGB8,
                                  normal_image8);
  this->normal_image_msg_ = *img_bridge.toImageMsg();
  this->normal_image_pub_->publish(this->normal_image_msg_);
}

void NpsGazeboRosMultibeamSonar::ComputePointCloud(const float *_src)
{
  // Use a lock guard for RAII-style thread safety.
  std::lock_guard<std::mutex> guard(this->lock_);

  // Set header information.
  this->point_cloud_msg_.header.frame_id = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nanosec = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

  // Use the PointCloud2Modifier to set fields and resize the message.
  sensor_msgs::PointCloud2Modifier pcd_modifier(this->point_cloud_msg_);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(this->height * this->width);

  // Resize (or create) the point cloud image (for example, to store point intensity).
  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);

  // Create iterators for the point cloud fields.
  sensor_msgs::PointCloud2Iterator<float> iter_x(this->point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(this->point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(this->point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(this->point_cloud_msg_, "rgb");
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();

  // Assume the point cloud is dense until a point is found out-of-range.
  this->point_cloud_msg_.is_dense = true;

  // Prepare to copy depth data from the source.
  float* toCopyFrom = const_cast<float*>(_src);
  int index = 0;

  // Compute the focal length from the horizontal field of view.
  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = static_cast<double>(this->width) / (2.0 * tan(hfov / 2.0));

  // Loop over each row (j) and column (i) in the image.
  for (uint32_t j = 0; j < this->height; j++)
  {
    double elevation;
    if (this->height > 1)
      elevation = atan2(static_cast<double>(j) - 0.5 * static_cast<double>(this->height), fl);
    else
      elevation = 0.0;

    // Store the computed elevation angle (if you use it elsewhere).
    this->elevation_angles[j] = static_cast<float>(elevation);

    for (uint32_t i = 0; i < this->width; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_image)
    {
      double azimuth;
      if (this->width > 1)
        azimuth = atan2(static_cast<double>(i) - 0.5 * static_cast<double>(this->width), fl);
      else
        azimuth = 0.0;

      double depth = toCopyFrom[index++];

      // For debugging purposes, print the depth if it is below a threshold.
      if (depth < 15.0)
      {
        RCLCPP_INFO(this->ros_node_->get_logger(), "Depth: %f", depth);
      }

      // Compute the x and y coordinates in the point cloud.
      *iter_x = depth * tan(azimuth);
      *iter_y = depth * tan(elevation);

      // If the depth exceeds the cutoff, record the z coordinate and compute the intensity.
      if (depth > this->point_cloud_cutoff_)
      {
        *iter_z = depth;
        *iter_image = std::sqrt((*iter_x) * (*iter_x) +
                                (*iter_y) * (*iter_y) +
                                (*iter_z) * (*iter_z));
      }
      else  // Otherwise mark the point as invalid.
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_image = 0.0f;
        this->point_cloud_msg_.is_dense = false;
      }

      // Fill in the rgb field for each point using data from an image.
      // Cast the image data to a pointer to uint8_t.
      uint8_t* image_src = reinterpret_cast<uint8_t*>(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == this->height * this->width * 3)
      {
        // If the image is color (3 channels), assign each channel.
        iter_rgb[0] = image_src[i * 3 + j * this->width * 3 + 0];
        iter_rgb[1] = image_src[i * 3 + j * this->width * 3 + 1];
        iter_rgb[2] = image_src[i * 3 + j * this->width * 3 + 2];
      }
      else if (this->image_msg_.data.size() == this->height * this->width)
      {
        // If the image is mono, copy the same value for each channel.
        iter_rgb[0] = image_src[i + j * this->width];
        iter_rgb[1] = image_src[i + j * this->width];
        iter_rgb[2] = image_src[i + j * this->width];
      }
      else
      {
        // Otherwise, default to zero.
        iter_rgb[0] = iter_rgb[1] = iter_rgb[2] = 0;
      }
    }
  }

  // Publish the point cloud only if there is at least one subscriber.
  if (this->point_cloud_connect_count_ > 0)
  {
    this->point_cloud_pub_->publish(this->point_cloud_msg_);
  }
  // The std::lock_guard automatically releases the lock when it goes out of scope.
}


/////////////////////////////////////////////////
// Precalculation of corrector for sonar calculation
void NpsGazeboRosMultibeamSonar::ComputeCorrector()
{
  double hFOV = this->parentSensor->DepthCamera()->HFOV().Radian();
  double hPixelSize = hFOV / this->width;
  double fl = static_cast<double>(this->width) / (2.0 * tan(hFOV / 2.0));

  // Reset the sum before accumulation.
  this->beamCorrectorSum = 0.0;

  // Beam culling correction precalculation
  for (size_t beam = 0; beam < nBeams; ++beam)
  {
    float beam_azimuthAngle = static_cast<float>(atan2(static_cast<double>(beam) - 0.5 * static_cast<double>(this->width), fl));
    for (size_t beam_other = 0; beam_other < nBeams; ++beam_other)
    {
      float beam_azimuthAngle_other = static_cast<float>(atan2(static_cast<double>(beam_other) - 0.5 * static_cast<double>(this->width), fl));
      float azimuthBeamPattern = unnormalized_sinc(static_cast<float>(M_PI * 0.884 / hPixelSize * sin(beam_azimuthAngle - beam_azimuthAngle_other)));
      // Store the absolute value of the beam pattern.
      this->beamCorrector[beam][beam_other] = std::abs(azimuthBeamPattern);
      // Accumulate the squared value.
      this->beamCorrectorSum += pow(azimuthBeamPattern, 2);
    }
  }
  // Take the square root of the sum to normalize.
  this->beamCorrectorSum = sqrt(this->beamCorrectorSum);
}

/////////////////////////////////////////////////
// Compute a normal image from a given depth image.
// Returns a cv::Mat with 3 channels, where the channels contain processed values.
cv::Mat NpsGazeboRosMultibeamSonar::ComputeNormalImage(cv::Mat &depth)
{
  // Define Sobel-like filters for derivative computation.
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) << 1,  2,  1,
                                                  0,  0,  0,
                                                 -1, -2, -1) / 8.0f;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) << 1,  0, -1,
                                                  2,  0, -2,
                                                  1,  0, -1) / 8.0f;

  // Flip filters to get the desired convolution kernels.
  cv::Mat f1m, f2m;
  cv::flip(f1, f1m, 0);
  cv::flip(f2, f2m, 1);

  cv::Mat n1, n2;
  // Compute gradients in each direction.
  cv::filter2D(depth, n1, -1, f1m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(depth, n2, -1, f2m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  // Determine regions with no valid depth readings.
  cv::Mat no_readings;
  cv::erode((depth == 0), no_readings, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  n1.setTo(0, no_readings);
  n2.setTo(0, no_readings);

  // Prepare a vector of images for merging into a 3-channel normal image.
  std::vector<cv::Mat> images(3);
  // Create an image filled with ones having the same dimensions as depth.
  cv::Mat ones = cv::Mat::ones(depth.rows, depth.cols, CV_32FC1);

  // Note: With different focal lengths, the expression might change.
  // Here we use: (n1, n2, (1/focal_length)*depth)
  images[0] = n1;                      // Green channel (or one component of the normal)
  images[1] = n2;                      // Red channel (or the second component)
  images[2] = (1.0f / this->focal_length_) * depth;  // Blue channel (scaled depth)

  cv::Mat normal_image;
  cv::merge(images, normal_image);

  // Normalize each 3D vector (pixel) in the normal image.
  for (int i = 0; i < normal_image.rows; ++i)
  {
    for (int j = 0; j < normal_image.cols; ++j)
    {
      cv::Vec3f &n = normal_image.at<cv::Vec3f>(i, j);
      // Normalize the vector (if non-zero).
      n = cv::normalize(n);
    }
  }
  return normal_image;
}

/////////////////////////////////////////////////
// Publish Camera Info
void NpsGazeboRosMultibeamSonar::PublishCameraInfo()
{
  RCLCPP_DEBUG_NAMED(this->ros_node_->get_logger(), "depth_camera",
                     "Publishing default camera info, then depth camera info");
  // Call the base class or utility function for publishing camera info.
  GazeboRosCameraUtils::PublishCameraInfo();

  // Only publish depth camera info if there are subscribers.
  if (this->depth_info_connect_count_ > 0)
  {
    // Get the current sensor update time.
    common::Time sensor_update_time = this->parentSensor->LastMeasurementTime();
    this->sensor_update_time_ = sensor_update_time;
    // Only update if the elapsed time since the last update exceeds the update period.
    if (sensor_update_time - this->last_depth_image_camera_info_update_time_ >= this->update_period_)
    {
      // Publish camera info via the designated publisher.
      this->PublishCameraInfo(this->depth_image_camera_info_pub_);
      // Update the last published time.
      this->last_depth_image_camera_info_update_time_ = sensor_update_time;
    }
  }
}

}
// Register the plugin
PLUGINLIB_EXPORT_CLASS(gazebo_ros::GazeboMultibeamSonarRasterBased, gazebo_ros::GazeboMultibeamSonarRasterBasedHeader)
