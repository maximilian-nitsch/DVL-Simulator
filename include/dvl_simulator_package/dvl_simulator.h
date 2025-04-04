/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <random>

#include "dvl_simulator_structures.h"

namespace dvl_simulator {

class DvlSimulator {
 public:
  // Default constructor
  DvlSimulator();

  // Constructor with parameters
  DvlSimulator(DvlSimParams dvlSimParams,
               DvlModelEnableSettings dvlModelEnableSettings, double dt,
               unsigned int seed);

  // Destructor
  ~DvlSimulator();

  // Measurement generation function
  DvlMeasurement generateDvlMeasurement(const Eigen::Vector3d& p_nb_n,
                                        const Eigen::Vector3d& v_nb_b,
                                        const Eigen::Vector3d& w_nb_b);

  // Reset simulator
  void resetSimulator();

  // Getter functions
  DvlSimParams getDvlSimParams() const;
  double getScaleFactor() const;
  Eigen::Vector4d getBeamBias() const;
  Eigen::Matrix<double, 3, 4> getJanusGeometryMatrix() const;
  Eigen::Matrix<double, 3, 4> getJanusGeometryMatrixPInv() const;
  DvlModelEnableSettings getDvlModelEnableSettings() const;
  bool getAltitudeLimitViolated() const;
  bool getVelocityLimitViolated() const;
  double getSampleTime() const;
  unsigned int getSeed() const;
  bool getUseFixedRandomNumbers() const;

  // Setter functions
  void setDvlSimParams(const DvlSimParams& dvlSimParams);
  void setDvlModelEnableSettings(
      const DvlModelEnableSettings& dvlModelEnableSettings);
  void setEnableBeamVelocityNoise(const bool enableBeamVelocityNoise);
  void setEnableScaleFactor(const bool enableScaleFactor);
  void setEnableBeamBias(const bool enableBeamBias);
  void setEnableGyroNoise(const bool enableGyroNoise);
  void setEnableQuantization(const bool enableQuantization);
  void setEnableAltitudeLimit(const bool enableAltitudeLimit);
  void setEnableVelocityLimit(const bool enableVelocityLimit);
  void setSampleTime(const double dt);
  void setSeed(const unsigned int seed);
  void setUseFixedRandomNumbers(const bool useFixedRandomNumbers);

  // Print function
  std::stringstream printDvlSimulatorParameters();

 private:
  // DVL simulation parameters
  DvlSimParams dvlSimParams_;

  // Internal state space variables
  double scaleFactor_;        // (%)
  Eigen::Vector4d beamBias_;  // (m/s)

  // Janus geometry matrix and its pseudoinverse
  Eigen::Matrix<double, 3, 4> janusGeometryMatrix_;
  Eigen::Matrix<double, 3, 4> janusGeometryMatrixPInv_;

  // DVL model enable settings
  DvlModelEnableSettings dvlModelEnableSettings_;

  // Flag indicating if constant errors have been initialized
  bool constErrorsInitialized_;

  // Flags indicating if limits are violated
  bool altitudeLimitViolated_;
  bool velocityLimitViolated_;

  // DVL sampling time
  double dt_;

  // Random number generator for normal distribution and uniform distribution
  std::mt19937 randomNumberGenerator_;
  unsigned int seed_;
  std::normal_distribution<> normalDistribution_;
  std::uniform_real_distribution<> uniformDistribution_;

  // Flag indicating if fixed random numbers are be used (for testing/debugging)
  bool useFixedRandomNumbers_;

  //  Geometry matrix calculation function
  void calculateJanusGeometryMatrix();

  // State space model update functions
  void updateScaleFactorRandomWalkModel();
  void updateBeamBiasGaussMarkovModel();

  // Quantization model
  Eigen::Vector3d calcQuantizationModel(const Eigen::Vector3d& measurement,
                                        const double resolution);

  // Helper function to calculate the skew-symmetric matrix of a 3x1 vector
  Eigen::Matrix3d calcSkewMatrix3(const Eigen::Vector3d& v);

  // Rancom constant initialization (turn-on bias, scale factor, misalignment)
  void initializeRandomConstantErrors();

  // Random number generation functions
  double drawRandNormalDistNum();
  double drawRandUniformDistNumFromInterval(const double interval);
};

}  // namespace dvl_simulator
