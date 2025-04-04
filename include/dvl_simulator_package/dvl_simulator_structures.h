/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>

namespace dvl_simulator {

struct DvlSimParams {
  double beamVelocityNoiseStdDev;           // Beam velocity STD (m/s)
  double intervalInitialScaleFactor;        // Interval initial scale factor (%)
  double scaleFactorStdDev;                 // Scale factor STD (%/sqrt(s))
  Eigen::Vector4d intervalInitialBeamBias;  // Interval initial beam bias (m/s)
  double biasStdDev;                        // Bias STD (m/s/sqrt(s))
  double biasCorrTime;                      // Bias correlation time (s)
  Eigen::Vector3d gyroStdDev;               // Gyroscope STD (rad/s)
  double resolution;                        // Velocity resolution (m/s)
  double beamAngle;                         // Janus beam angle (deg)
  double minAltitude;                       // Min. working altitude(m)
  double maxAltitude;                       // Max. working altitude (m)
  double downDistanceToGround;              // Distance to ground (m)
  double maxVelocity;                       // Max. working velocity (m/s)
  Eigen::Vector3d p_bs_b;                   // Lever arm (m)
  Eigen::Matrix3d C_s_b;                    // Rotation matrix (-)
};

struct DvlMeasurement {
  Eigen::Vector4d beamVelocities;  // Beam velocities (m/s)
  Eigen::Vector3d velocity;        // Velocity (m/s)
  Eigen::Matrix3d covariance;      // Velocity covariance (m/s)^2
  bool velocityValid;              // Measurement valid flag
};

struct DvlModelEnableSettings {
  bool enableBeamVelocityNoise;  // Enable beam velocity noise
  bool enableScaleFactor;        // Enable scale factor
  bool enableBeamBias;           // Enable beam bias
  bool enableGyroNoise;          // Enable gyroscope noise
  bool enableQuantization;       // Enable quantization
  bool enableAltitudeLimit;      // Enable altitude limit
  bool enableVelocityLimit;      // Enable velocity limit
};

}  // namespace dvl_simulator
