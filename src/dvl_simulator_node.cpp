/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "nanoauv_sensor_driver_interfaces/msg/dvl_beam.hpp"
#include "nanoauv_sensor_driver_interfaces/msg/dvl_velocity.hpp"

#include "dvl_simulator.h"

namespace dvl_simulator {

/**
 * @brief DVL simulator node class.
 * 
 * This class is the main node class of the DVL simulator package. It is
 * responsible for the initialization of the DVL simulator object, the
 * subscription to the ground truth odometry topic, and the publication of the
 * simulated DVL custom data message.
 * 
*/
class DvlSimulatorNode : public rclcpp::Node {
 public:
  // Constructor with default config file path
  explicit DvlSimulatorNode(std::shared_ptr<DvlSimulator> pDvlSimulator);

  // Destructor
  ~DvlSimulatorNode() {}

 private:
  // DVL simulator class object
  std::shared_ptr<DvlSimulator> pDvlSimulator_;

  // Ground truth odometry message
  nav_msgs::msg::Odometry::SharedPtr groundTruthOdomMsg_;

  // Last odometry timestamp
  rclcpp::Time lastOdomTimestamp_;

  // Custom DVL velocity message publisher
  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::DvlVelocity>::
      SharedPtr pCustomDvlVelocityPublisherPtr_;

  // Vehicle odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pOdometrySubscriber_;

  // Static tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;

  // Timers
  rclcpp::TimerBase::SharedPtr pTimer_;
  rclcpp::TimerBase::SharedPtr pOdometryTimeOutTimer_;

  // Sample time
  double sampleTime_;

  // Time since last velocity report
  rclcpp::Time timeLastVelocityReport_;

  // Distance to ground
  double distanceToGround_;

  // Odometry flags
  bool first_odometry_received_;
  bool odometry_timeout_;

  // Declaration and retrieval functions for parameters from YAML file
  void declareAndRetrieveGeneralSettings();
  void declareAndRetrieveDvlParameters();
  void declareAndRetrieveEnableSettings();

  // Timer callback function
  void dvlSimulatorLoopCallback();

  // Odometry callback functions
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometryTimeOutCallback();

  // tf2 static broadcaster callback function
  void publishStaticTf2Transforms();

  // Helper functions
  Eigen::Vector3d doubleVectorToEigenVector3(const std::vector<double>& vec);

  // Node namespace
  std::string node_namespace_;
};

/**
 * @brief DVL simulator node constructor with default config file path.
 * 
 * @param[in] pDvlSimulator Pointer to the DVL simulator object
*/
DvlSimulatorNode::DvlSimulatorNode(std::shared_ptr<DvlSimulator> pDvlSimulator)
    : Node("dvl_simulator_node"),
      pDvlSimulator_(pDvlSimulator),
      lastOdomTimestamp_(0, 0),
      sampleTime_(0.2),
      timeLastVelocityReport_(now()),
      first_odometry_received_(false),
      odometry_timeout_(false),
      node_namespace_(this->get_namespace()) {

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Configuring DVL simulator node...");

  // Declare and retrieve parameters and load them to DVL simulator
  declareAndRetrieveGeneralSettings();
  declareAndRetrieveDvlParameters();
  declareAndRetrieveEnableSettings();

  // Reset DVL simulator to ensure that constant errors are initialized
  pDvlSimulator_->resetSimulator();

  // Print DVL simulator parameters
  std::stringstream ss = pDvlSimulator_->printDvlSimulatorParameters();
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_), "%s", ss.str().c_str());

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Parameters from YAML config loaded successfully.");

  // Declare the value of the topic_name parameter
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);

  // Retrieve topic name from launch file or use default
  std::string groundTruthTopicName;
  this->get_parameter_or("topic_name", groundTruthTopicName,
                         std::string("/nanoauv/odometry"));

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Subscribing to ground truth odometry topic: %s",
              groundTruthTopicName.c_str());

  // Initialize the odometry subscriber
  pOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      groundTruthTopicName, 10,
      std::bind(&DvlSimulatorNode::odometryCallback, this,
                std::placeholders::_1));

  // Initialize the DVL custom velocity msg publisher
  pCustomDvlVelocityPublisherPtr_ = this->create_publisher<
      nanoauv_sensor_driver_interfaces::msg::DvlVelocity>(
      node_namespace_ + "/velocity_and_transducer", 10);

  // Initialize the tf2 broadcaster
  pStaticTf2Broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Publish static tf2 transformations
  this->publishStaticTf2Transforms();

  // Extract DVL sample time from the parameter server
  double sampleTime =
      get_parameter("dvl_simulator.general_settings.sample_time").as_double();

  // Set class member sample time
  sampleTime_ = sampleTime;

  // Convert sample time to milliseconds and cast to int
  int sampleTimeInt = static_cast<int>(sampleTime * 1e3);

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "DVL simulator node executing with %dms.", sampleTimeInt);

  // Create a timer to call the DVL simulator loop callback function
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(sampleTimeInt),
      std::bind(&DvlSimulatorNode::dvlSimulatorLoopCallback, this));

  // Create timer for timeout
  pOdometryTimeOutTimer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&DvlSimulatorNode::odometryTimeOutCallback, this));

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "DVL simulator node initialized. DVL simulator node waiting for "
              "first odometry message...");
}

/**
 * @brief Declare and retrieve general settings from the parameter server.
*/
void DvlSimulatorNode::declareAndRetrieveGeneralSettings() {
  // General settings
  double sampleTime;
  int seed;
  bool useConstantSeed;

  // Declare general settings
  this->declare_parameter("dvl_simulator.general_settings.sample_time",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dvl_simulator.general_settings.seed",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("dvl_simulator.general_settings.use_constant_seed",
                          rclcpp::PARAMETER_BOOL);

  // Retrieve general settings
  sampleTime = this->get_parameter("dvl_simulator.general_settings.sample_time")
                   .as_double();
  seed = this->get_parameter("dvl_simulator.general_settings.seed").as_int();
  useConstantSeed =
      this->get_parameter("dvl_simulator.general_settings.use_constant_seed")
          .as_bool();

  // Set DVL simulator sample time and seed
  pDvlSimulator_->setSampleTime(sampleTime);

  // Set seed depending on the useConstantSeed flag
  if (useConstantSeed == false) {
    // Draw a random seed from the random device
    std::random_device rd;
    seed = rd();
    pDvlSimulator_->setSeed(seed);
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_), "Using random seed: %d",
                seed);
  } else {
    // Set the random number generator seed
    pDvlSimulator_->setSeed(seed);
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Using seed from config file: %d", seed);
  }
}

/**
 * @brief Declare and retrieve DVL model parameters from the parameter server.
*/
void DvlSimulatorNode::declareAndRetrieveDvlParameters() {
  // DVL model parameters
  DvlSimParams dvlSimParams;

  // Declare model parameters
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.beam_velocity_noise_std",
      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.scale_factor_noise_std",
      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.beam_bias_noise_std",
      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.gyroscope_noise_std",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.interval_initial_scale_factor",
      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.interval_initial_beam_bias",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.beam_bias_correlation_time",
      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dvl_simulator.model_parameter_settings.resolution",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dvl_simulator.model_parameter_settings.beam_angle",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dvl_simulator.model_parameter_settings.min_altitude",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dvl_simulator.model_parameter_settings.max_altitude",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.distance_to_ground",
      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dvl_simulator.model_parameter_settings.max_velocity",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.lever_arm_body_to_sensor_frame",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "dvl_simulator.model_parameter_settings.rotation_body_to_sensor_frame",
      rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Retrieve model parameters
  double beamVelocityNoiseStd =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.beam_velocity_noise_std")
          .as_double();
  double scaleFactorNoiseStd =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.scale_factor_noise_std")
          .as_double();
  double beamBiasNoiseStd =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.beam_bias_noise_std")
          .as_double();
  std::vector<double> gyroscopeNoiseStd =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.gyroscope_noise_std")
          .as_double_array();
  double intervalInitialScaleFactor =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.interval_initial_scale_"
              "factor")
          .as_double();
  std::vector<double> intervalInitialBeamBias =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.interval_initial_beam_"
              "bias")
          .as_double_array();
  double beamBiasCorrelationTime =
      this->get_parameter(
              "dvl_simulator.model_parameter_"
              "settings.beam_bias_correlation_time")
          .as_double();
  double resolution =
      this->get_parameter("dvl_simulator.model_parameter_settings.resolution")
          .as_double();
  double beamAngle =
      this->get_parameter("dvl_simulator.model_parameter_settings.beam_angle")
          .as_double();
  double minAltitude =
      this->get_parameter("dvl_simulator.model_parameter_settings.min_altitude")
          .as_double();
  double maxAltitude =
      this->get_parameter("dvl_simulator.model_parameter_settings.max_altitude")
          .as_double();
  double distanceToGround =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.distance_to_ground")
          .as_double();
  double maxVelocity =
      this->get_parameter("dvl_simulator.model_parameter_settings.max_velocity")
          .as_double();
  std::vector<double> leverArmBodyToSensorFrame =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.lever_arm_body_to_sensor_"
              "frame")
          .as_double_array();
  std::vector<double> rotationBodyToSensorFrame =
      this->get_parameter(
              "dvl_simulator.model_parameter_settings.rotation_body_to_sensor_"
              "frame")
          .as_double_array();

  // Assign model parameters to struct
  dvlSimParams.beamVelocityNoiseStdDev = beamVelocityNoiseStd;
  dvlSimParams.scaleFactorStdDev = scaleFactorNoiseStd;
  dvlSimParams.biasStdDev = beamBiasNoiseStd;
  dvlSimParams.gyroStdDev = doubleVectorToEigenVector3(gyroscopeNoiseStd);
  dvlSimParams.intervalInitialScaleFactor = intervalInitialScaleFactor;
  dvlSimParams.intervalInitialBeamBias << intervalInitialBeamBias[0],
      intervalInitialBeamBias[1], intervalInitialBeamBias[2],
      intervalInitialBeamBias[3];
  dvlSimParams.biasCorrTime = beamBiasCorrelationTime;
  dvlSimParams.resolution = resolution;
  dvlSimParams.beamAngle = beamAngle;
  dvlSimParams.minAltitude = minAltitude;
  dvlSimParams.maxAltitude = maxAltitude;
  dvlSimParams.downDistanceToGround = distanceToGround;
  dvlSimParams.maxVelocity = maxVelocity;
  dvlSimParams.p_bs_b = doubleVectorToEigenVector3(leverArmBodyToSensorFrame);

  // Extract sensor rotation as Euler angles in ZYX order from parameter server
  Eigen::Vector3d sensorRotationEulerRpy =
      doubleVectorToEigenVector3(rotationBodyToSensorFrame) * M_PI /
      180.0;  // deg to rad

  // Convert Euler angles to rotation matrix
  dvlSimParams.C_s_b =
      Eigen::AngleAxisd(sensorRotationEulerRpy[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(sensorRotationEulerRpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(sensorRotationEulerRpy[2], Eigen::Vector3d::UnitX());

  // Set DVL simulator parameters
  pDvlSimulator_->setDvlSimParams(dvlSimParams);

  // Set DVL simulator node distance to ground
  distanceToGround_ = distanceToGround;
}

/**
 * @brief Declare and retrieve DVL model enable settings from the parameter server.
*/
void DvlSimulatorNode::declareAndRetrieveEnableSettings() {
  // Model enable settings
  DvlModelEnableSettings dvlModelEnableSettings;

  // Model enable settings
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_beam_velocity_noise",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_scale_factor",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_beam_bias",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_gyroscope_noise",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_quantization",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_altitude_limit",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "dvl_simulator.model_enable_settings.enable_velocity_limit",
      rclcpp::PARAMETER_BOOL);

  // Retrieve model enable settings
  bool enableBeamVelocityNoise = this->get_parameter(
                                         "dvl_simulator.model_enable_settings."
                                         "enable_beam_velocity_noise")
                                     .as_bool();
  bool enableScaleFactor = this->get_parameter(
                                   "dvl_simulator.model_enable_settings."
                                   "enable_scale_factor")
                               .as_bool();
  bool enableBeamBias = this->get_parameter(
                                "dvl_simulator.model_enable_settings."
                                "enable_beam_bias")
                            .as_bool();
  bool enableGyroNoise = this->get_parameter(
                                 "dvl_simulator.model_enable_settings."
                                 "enable_gyroscope_noise")
                             .as_bool();
  bool enableQuantization = this->get_parameter(
                                    "dvl_simulator.model_enable_settings."
                                    "enable_quantization")
                                .as_bool();
  bool enableAltitudeLimit = this->get_parameter(
                                     "dvl_simulator.model_enable_settings."
                                     "enable_altitude_limit")
                                 .as_bool();
  bool enableVelocityLimit = this->get_parameter(
                                     "dvl_simulator.model_enable_settings."
                                     "enable_velocity_limit")
                                 .as_bool();

  // Assign model enable settings to struct
  dvlModelEnableSettings.enableBeamVelocityNoise = enableBeamVelocityNoise;
  dvlModelEnableSettings.enableScaleFactor = enableScaleFactor;
  dvlModelEnableSettings.enableBeamBias = enableBeamBias;
  dvlModelEnableSettings.enableGyroNoise = enableGyroNoise;
  dvlModelEnableSettings.enableQuantization = enableQuantization;
  dvlModelEnableSettings.enableAltitudeLimit = enableAltitudeLimit;
  dvlModelEnableSettings.enableVelocityLimit = enableVelocityLimit;

  // Set DVL simulator enable settings
  pDvlSimulator_->setDvlModelEnableSettings(dvlModelEnableSettings);
}

/**
 * @brief DVL simulator loop callback function.
 * 
 * This function is called periodically by the timer and publishes the
 * custom DVL message. The DVL simulator is updated with ground truth
 * data and the simulated DVL data is published.
 * 
 */
void DvlSimulatorNode::dvlSimulatorLoopCallback() {
  // Get current time
  rclcpp::Time currentTimestamp = now();

  // Create ROS messages to be published
  nanoauv_sensor_driver_interfaces::msg::DvlVelocity dvlCustomVelocityMsg;

  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  // ****************  Initialize dvlCustomVelocityMsg  *************** //
  dvlCustomVelocityMsg.header.stamp = currentTimestamp;
  dvlCustomVelocityMsg.header.frame_id = "dvl_link";

  dvlCustomVelocityMsg.time_since_last_velocity_report.data = 0.0;

  dvlCustomVelocityMsg.velocity.x = 0.0;
  dvlCustomVelocityMsg.velocity.y = 0.0;
  dvlCustomVelocityMsg.velocity.z = 0.0;

  dvlCustomVelocityMsg.figure_of_merit.data = 0.0;

  for (int i = 0; i < 9; i++) {
    dvlCustomVelocityMsg.velocity_covariance[i].data = 0.0;
  }

  dvlCustomVelocityMsg.altitude.data =
      9999999.0;  // Initialize with invalid value

  for (int i = 0; i < 4; i++) {
    dvlCustomVelocityMsg.transducers[0].transducer_id.data = 0;
    dvlCustomVelocityMsg.transducers[0].beam_velocity.data = 0.0;
    dvlCustomVelocityMsg.transducers[0].range_to_reflecting_surface.data =
        9999999.0;  // Initialize with invalid value
    dvlCustomVelocityMsg.transducers[0]
        .received_signal_strength_indicator.data =
        9999999.0;  // Initialize with invalid value
    dvlCustomVelocityMsg.transducers[0].noise_spectral_density.data =
        9999999.0;  // Initialize with invalid value
    dvlCustomVelocityMsg.transducers[0].is_valid.data = false;
  }

  dvlCustomVelocityMsg.is_valid.data = false;
  dvlCustomVelocityMsg.status.data = 0;
  dvlCustomVelocityMsg.time_of_surface_reflection.sec = 0;
  dvlCustomVelocityMsg.time_of_surface_reflection.nanosec = 0;
  dvlCustomVelocityMsg.time_of_transmission.sec = 0;
  dvlCustomVelocityMsg.time_of_transmission.nanosec = 0;
  // ****************************************************************** //

  // Read out odometry message
  Eigen::Vector3d p_nb_n_true;
  Eigen::Vector3d v_nb_b_true;
  Eigen::Vector3d w_nb_b_true;

  // Check if ground truth odometry message is available
  if (groundTruthOdomMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth odometry message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "DVL Simulator";
    diagnosticMsg.message = "Waiting for first ground truth odometry message!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic message to diagnostic array message
    dvlCustomVelocityMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish the custom DVL velocity message with only diagnostic message
    pCustomDvlVelocityPublisherPtr_->publish(dvlCustomVelocityMsg);

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    return;

  } else {
    // Assign ground truth odometry message to DVL simulator inputs
    p_nb_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.position.x;
    p_nb_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.position.y;
    p_nb_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.position.z;

    v_nb_b_true.x() = groundTruthOdomMsg_.get()->twist.twist.linear.x;
    v_nb_b_true.y() = groundTruthOdomMsg_.get()->twist.twist.linear.y;
    v_nb_b_true.z() = groundTruthOdomMsg_.get()->twist.twist.linear.z;

    w_nb_b_true.x() = groundTruthOdomMsg_.get()->twist.twist.angular.x;
    w_nb_b_true.y() = groundTruthOdomMsg_.get()->twist.twist.angular.y;
    w_nb_b_true.z() = groundTruthOdomMsg_.get()->twist.twist.angular.z;

    // Generate DVL measurement
    DvlMeasurement dvlMeasurement = pDvlSimulator_->generateDvlMeasurement(
        p_nb_n_true, v_nb_b_true, w_nb_b_true);

    // Calculate time since last velocity report
    rclcpp::Duration timeSinceLastVelocityReport =
        currentTimestamp - timeLastVelocityReport_;

    // Save time of last velocity report
    timeLastVelocityReport_ = currentTimestamp;

    // Fill twist field message header
    dvlCustomVelocityMsg.header.stamp = currentTimestamp;
    dvlCustomVelocityMsg.header.frame_id = "dvl_link";

    // Fill custom velocity message with simulated velocity data
    dvlCustomVelocityMsg.velocity.x = dvlMeasurement.velocity(0);
    dvlCustomVelocityMsg.velocity.y = dvlMeasurement.velocity(1);
    dvlCustomVelocityMsg.velocity.z = dvlMeasurement.velocity(2);

    // Fill custom velocity message with simulated covariance data
    dvlCustomVelocityMsg.velocity_covariance.at(0).data =
        dvlMeasurement.covariance(0, 0);
    dvlCustomVelocityMsg.velocity_covariance.at(1).data =
        dvlMeasurement.covariance(0, 1);
    dvlCustomVelocityMsg.velocity_covariance.at(2).data =
        dvlMeasurement.covariance(0, 2);

    dvlCustomVelocityMsg.velocity_covariance.at(3).data =
        dvlMeasurement.covariance(1, 0);
    dvlCustomVelocityMsg.velocity_covariance.at(4).data =
        dvlMeasurement.covariance(1, 1);
    dvlCustomVelocityMsg.velocity_covariance.at(5).data =
        dvlMeasurement.covariance(1, 2);

    dvlCustomVelocityMsg.velocity_covariance.at(6).data =
        dvlMeasurement.covariance(2, 0);
    dvlCustomVelocityMsg.velocity_covariance.at(7).data =
        dvlMeasurement.covariance(2, 1);
    dvlCustomVelocityMsg.velocity_covariance.at(8).data =
        dvlMeasurement.covariance(2, 2);

    // Fill the four beam messages
    for (int i = 0; i < 4; i++) {
      dvlCustomVelocityMsg.transducers[i].transducer_id.data = i;
      dvlCustomVelocityMsg.transducers[i].beam_velocity.data =
          dvlMeasurement.beamVelocities(i);
      dvlCustomVelocityMsg.transducers[i].range_to_reflecting_surface.data =
          9999999.0;
      dvlCustomVelocityMsg.transducers[i]
          .received_signal_strength_indicator.data = 9999999.0;
      dvlCustomVelocityMsg.transducers[i].noise_spectral_density.data =
          9999999.0;
      dvlCustomVelocityMsg.transducers[i].is_valid.data = true;
    }

    // Set the valid flag to true
    dvlCustomVelocityMsg.is_valid.data = true;

    // Fill time of surface reflection and transmission with current time
    dvlCustomVelocityMsg.time_of_surface_reflection.sec =
        currentTimestamp.seconds();
    dvlCustomVelocityMsg.time_of_surface_reflection.nanosec =
        currentTimestamp.nanoseconds();

    dvlCustomVelocityMsg.time_of_transmission.sec = currentTimestamp.seconds();
    dvlCustomVelocityMsg.time_of_transmission.nanosec =
        currentTimestamp.nanoseconds();

    // Fill the time since last velocity report field message
    dvlCustomVelocityMsg.time_since_last_velocity_report.data =
        timeSinceLastVelocityReport.seconds();

    // Fill the figure of merit field message placeholder data
    dvlCustomVelocityMsg.figure_of_merit.data = 9999999.0;

    // Fill altitude field message with simulated data
    dvlCustomVelocityMsg.altitude.data = p_nb_n_true.z() - distanceToGround_;

    // Fill the diagnostic message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = "DVL Simulator";
    diagnosticMsg.message = "DVL simulator running nominal.";

    // Check if altitude limit is violated
    if (pDvlSimulator_->getAltitudeLimitViolated() == true) {
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnosticMsg.name = "DVL Simulator";
      diagnosticMsg.message = "Altitude limit violated!";

      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(node_namespace_), *get_clock(),
                           5000, "Altitude limit violated!");
    }

    // Check if velocity limit is violated
    if (pDvlSimulator_->getVelocityLimitViolated() == true) {
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnosticMsg.name = "DVL Simulator";
      diagnosticMsg.message = "Velocity limit violated!";

      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(node_namespace_), *get_clock(),
                           5000, "Velocity limit violated!");
    }

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic message to custom DVL velocity message
    dvlCustomVelocityMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish the custom DVL velocity message with only diagnostic message
    pCustomDvlVelocityPublisherPtr_->publish(dvlCustomVelocityMsg);
  }

  // Calculate time since last odometry message
  rclcpp::Duration timeSinceLastOdom =
      rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

  if (timeSinceLastOdom.seconds() > sampleTime_ && odometry_timeout_ == false) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnosticMsg.name = "DVL Simulator";
    diagnosticMsg.message =
        "Ground truth odometry message frequency is too slow!"
        " DVL simulator ground truth frequency higher than odometry!"
        " Increase odometry message frequency!";

    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(node_namespace_), *get_clock(), 5000,
        "Ground truth odometry message frequency is too slow!");
  }

  if (odometry_timeout_ == true) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "DVL Simulator";
    diagnosticMsg.message =
        "No ground truth odometry message received since than 5 seconds!"
        " DVL simulator stalling!";

    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(node_namespace_), *get_clock(), 5000,
        "No ground truth odometry message since more than 5 seconds!");
  }

  // Add diagnostic message to diagnostic array message
  diagnosticArrayMsg.status.push_back(diagnosticMsg);
  diagnosticArrayMsg.header.stamp = currentTimestamp;

  // Add diagnostic message to custom DVL velocity message
  dvlCustomVelocityMsg.diagnostic_array = diagnosticArrayMsg;

  // Publish the custom DVL velocity message with only diagnostic message
  pCustomDvlVelocityPublisherPtr_->publish(dvlCustomVelocityMsg);
}

/**
 * @brief Odometry callback function.
 * 
 * This function is called when a new odometry message is received. The ground
 * truth odometry message is assigned to the ground truth odometry message
 * member variable of the node class.
 * 
 * @param[in] msg Pointer to the odometry message
*/
void DvlSimulatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Reset odometry timeout timer
  pOdometryTimeOutTimer_->cancel();
  pOdometryTimeOutTimer_->reset();

  // Set first odometry received flag
  if (first_odometry_received_ == false) {
    first_odometry_received_ = true;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "First ground truth odometry message received! DVL simulator "
                "now running nominal!");
  }
  // Reset odometry timeout flag
  if (odometry_timeout_ == true) {
    odometry_timeout_ = false;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Ground truth odometry message received after timeout! DVL "
                "simulator now running nominal!");
  }

  // Assign ground truth odometry message
  groundTruthOdomMsg_ = msg;

  // Assign last odometry timestamp
  lastOdomTimestamp_ = msg->header.stamp;
}

/**
 * @brief Odometry timeout callback function.
 * 
 * This function is called when no ground truth odometry message is received.
 * 
*/
void DvlSimulatorNode::odometryTimeOutCallback() {
  // Set odometry timeout flag
  odometry_timeout_ = true;

  RCLCPP_WARN_THROTTLE(rclcpp::get_logger(node_namespace_), *get_clock(), 5000,
                       "No ground truth odometry message since more than 5 "
                       "seconds! DVL simulator now starting to stale!");
}

/**
 * @brief Publish static tf2 transformations.
*/
void DvlSimulatorNode::publishStaticTf2Transforms() {
  // Get DVL simulator parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator_->getDvlSimParams();

  // Convert rotation matrix to quaternion (sensor to body frame)
  Eigen::Quaterniond q_s_b = Eigen::Quaterniond(dvlSimParams.C_s_b);

  // Fill tf2 transform message between base_link (body) and dvl_link (sensor)
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = now();

  tfMsg.header.frame_id = "base_link_sname";
  tfMsg.child_frame_id = "dvl_link";

  tfMsg.transform.translation.x = dvlSimParams.p_bs_b[0];
  tfMsg.transform.translation.y = dvlSimParams.p_bs_b[1];
  tfMsg.transform.translation.z = dvlSimParams.p_bs_b[2];

  tfMsg.transform.rotation.w = q_s_b.w();
  tfMsg.transform.rotation.x = q_s_b.y();
  tfMsg.transform.rotation.y = q_s_b.x();
  tfMsg.transform.rotation.z = q_s_b.z();

  pStaticTf2Broadcaster_->sendTransform(tfMsg);
}

/** 
 * @brief Helper function to convert a vector of doubles to an Eigen vector.
 * 
 * @param[in] vec Vector of doubles
 * @return Eigen vector
*/
Eigen::Vector3d DvlSimulatorNode::doubleVectorToEigenVector3(
    const std::vector<double>& vec) {
  // Convert vector of doubles to Eigen vector
  Eigen::Vector3d eigenVec;

  // Check vector size
  if (vec.size() != 3) {
    RCLCPP_ERROR(rclcpp::get_logger(node_namespace_),
                 "doubleVectorToEigenVector3: Vector size is not 3!");
  } else {
    eigenVec << vec[0], vec[1], vec[2];
  }

  return eigenVec;
}

}  // namespace dvl_simulator

/**
 * @brief Main function of the DVL simulator node.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 *
 * @return int Return value
 */
int main(int argc, char** argv) {
  // Create DVL simulator object
  std::shared_ptr<dvl_simulator::DvlSimulator> pDvlSimulator =
      std::make_shared<dvl_simulator::DvlSimulator>();

  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<dvl_simulator::DvlSimulatorNode>(pDvlSimulator));
  rclcpp::shutdown();

  return 0;
}
