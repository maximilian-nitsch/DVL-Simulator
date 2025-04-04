/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <memory>

#include "dvl_simulator.h"

#include "gtest/gtest.h"

/**
 * @brief Test fixture for the DVL simulator class.
*/
class DvlSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Fill the DVL simulation parameters
    dvlSimParams.beamVelocityNoiseStdDev = 0.01;    // (m/s)
    dvlSimParams.intervalInitialScaleFactor = 0.1;  // (%)
    dvlSimParams.scaleFactorStdDev = 0.0;           // (%/sqrt(s))
    dvlSimParams.intervalInitialBeamBias =
        Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);  // (m/s)
    dvlSimParams.biasStdDev = 5e-04;          // (m/s/sqrt(s))
    dvlSimParams.biasCorrTime = 1800.0;       // (s)
    dvlSimParams.gyroStdDev =
        Eigen::Vector3d(0.000207479860047933, 0.000243411420514079,
                        0.000187943045943727);             // (rad/s)
    dvlSimParams.resolution = 1e-04;                       // (m/s)
    dvlSimParams.beamAngle = 22.5;                         // (deg)
    dvlSimParams.minAltitude = 0.05;                       // (m)
    dvlSimParams.maxAltitude = 50.0;                       // (m)
    dvlSimParams.downDistanceToGround = -5.0;              // (m)
    dvlSimParams.maxVelocity = 3.75;                       // (m/s)
    dvlSimParams.p_bs_b = Eigen::Vector3d(0.0, 0.0, 0.0);  // (m)
    dvlSimParams.C_s_b = Eigen::Matrix3d::Identity();      // (-)

    // Fill the DVL model enable settings
    dvlModelEnableSettings.enableBeamVelocityNoise = false;
    dvlModelEnableSettings.enableScaleFactor = false;
    dvlModelEnableSettings.enableBeamBias = false;
    dvlModelEnableSettings.enableGyroNoise = false;
    dvlModelEnableSettings.enableQuantization = false;
    dvlModelEnableSettings.enableAltitudeLimit = false;

    // DVL sample time
    double sampleTime = 0.2;

    // Random number generator seed
    unsigned int seed = 42;

    // Initialize the DVL simulator class
    pDvlSimulator = std::make_unique<dvl_simulator::DvlSimulator>(
        dvlSimParams, dvlModelEnableSettings, sampleTime, seed);
  }

  // Simulation parameters for WaterLinked DVL A50
  dvl_simulator::DvlSimParams dvlSimParams;

  // DVL model enable settings
  dvl_simulator::DvlModelEnableSettings dvlModelEnableSettings;

  // Declare the class under test
  std::unique_ptr<dvl_simulator::DvlSimulator> pDvlSimulator;
};

/**
 * @brief Test the reset function of the DVL simulator.
*/
TEST_F(DvlSimulatorTest, ResetSimulatorTest) {
  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator->getDvlSimParams();

  // Set the DVL simulation parameters
  dvlSimParams.intervalInitialScaleFactor = 0.75;  // (%)
  dvlSimParams.scaleFactorStdDev = 5.0;            // (%/sqrt(s))
  dvlSimParams.intervalInitialBeamBias =
      Eigen::Vector4d(1.0, 2.0, 3.0, 4.0);  // (m/s)
  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Get the DVL enable settings
  dvl_simulator::DvlModelEnableSettings dvlModelEnableSettings =
      pDvlSimulator->getDvlModelEnableSettings();

  // Set the DVL enable settings
  dvlModelEnableSettings.enableScaleFactor = true;
  dvlModelEnableSettings.enableBeamBias = true;

  pDvlSimulator->setDvlModelEnableSettings(dvlModelEnableSettings);

  // Test for 1000 resets with random scale factors and beam biases
  for (int i = 0; i < 1000; i++) {
    // Get the DVL scale factor and beam bias before reset
    double scaleFactorBeforeReset = pDvlSimulator->getScaleFactor();
    Eigen::Vector4d beamBiasBeforeReset = pDvlSimulator->getBeamBias();

    // Reset the simulator
    pDvlSimulator->resetSimulator();

    // Get the DVL scale factor and beam bias after reset
    double scaleFactorAfterReset = pDvlSimulator->getScaleFactor();
    Eigen::Vector4d beamBiasAfterReset = pDvlSimulator->getBeamBias();

    // Compare the scale factor before and after reset
    EXPECT_TRUE(std::fabs(scaleFactorBeforeReset - scaleFactorAfterReset) >
                1e-12);

    // Compare the beam bias before and after reset
    for (int i = 0; i < 4; i++) {
      EXPECT_TRUE(std::fabs(beamBiasBeforeReset(i) - beamBiasAfterReset(i)) >
                  1e-12);
    }

    // Generate a DVL measurement
    Eigen::Vector3d p_nb_n = Eigen::Vector3d(1.0, 2.0, 3.0);
    Eigen::Vector3d v_nb_b = Eigen::Vector3d(4.0, 5.0, 6.0);
    Eigen::Vector3d w_nb_b = Eigen::Vector3d(7.0, 8.0, 9.0);

    pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

    // Get the DVL scale factor and beam bias after measurement generation
    double scaleFactorAfterMeasGen = pDvlSimulator->getScaleFactor();
    Eigen::Vector4d beamBiasAfterMeasGen = pDvlSimulator->getBeamBias();

    // Compare the scale factor before and after measurement generation
    EXPECT_TRUE(std::fabs(scaleFactorAfterReset - scaleFactorAfterMeasGen) >
                1e-12);

    // Compare the beam bias before and after measurement generation
    for (int i = 0; i < 4; i++) {
      EXPECT_TRUE(std::fabs(beamBiasAfterReset(i) - beamBiasAfterMeasGen(i)) >
                  1e-12);
    }
  }
}

/**
 * @brief Test the getter and setter for the DVL simulation parameters.
*/
TEST_F(DvlSimulatorTest, DvlSimParamsGetterSetterTest) {
  // Fill the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams;
  dvlSimParams.beamVelocityNoiseStdDev = 1.0;     // (m/s)
  dvlSimParams.intervalInitialScaleFactor = 2.0;  // (%)
  dvlSimParams.scaleFactorStdDev = 3.0;           // (%/sqrt(s))
  dvlSimParams.intervalInitialBeamBias =
      Eigen::Vector4d(4.0, 5.0, 6.0, 7.0);                      // (m/s)
  dvlSimParams.biasStdDev = 8.0;                                // (m/s/sqrt(s))
  dvlSimParams.biasCorrTime = 9.0;                              // (s)
  dvlSimParams.gyroStdDev = Eigen::Vector3d(10.0, 11.0, 12.0);  // (rad/s)
  dvlSimParams.resolution = 13.0;                               // (m/s)
  dvlSimParams.beamAngle = 14.0;                                // (deg)
  dvlSimParams.minAltitude = 15.0;                              // (m)
  dvlSimParams.maxAltitude = 16.0;                              // (m)
  dvlSimParams.downDistanceToGround = 17.0;                     // (m)
  dvlSimParams.maxVelocity = 18.0;                              // (m/s)
  dvlSimParams.p_bs_b = Eigen::Vector3d(19.0, 20.0, 21.0);      // (m)
  dvlSimParams.C_s_b << 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0,
      30.0;  // (-)

  // Set the DVL simulation parameters
  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParamsReturned =
      pDvlSimulator->getDvlSimParams();

  // Compare expected with returned DVL simulation parameters
  EXPECT_NEAR(dvlSimParams.beamVelocityNoiseStdDev,
              dvlSimParamsReturned.beamVelocityNoiseStdDev, 1e-12);
  EXPECT_NEAR(dvlSimParams.intervalInitialScaleFactor,
              dvlSimParamsReturned.intervalInitialScaleFactor, 1e-12);
  EXPECT_NEAR(dvlSimParams.scaleFactorStdDev,
              dvlSimParamsReturned.scaleFactorStdDev, 1e-12);
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(dvlSimParams.intervalInitialBeamBias(i),
                dvlSimParamsReturned.intervalInitialBeamBias(i), 1e-12);
  }
  EXPECT_NEAR(dvlSimParams.biasStdDev, dvlSimParamsReturned.biasStdDev, 1e-12);
  EXPECT_NEAR(dvlSimParams.biasCorrTime, dvlSimParamsReturned.biasCorrTime,
              1e-12);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(dvlSimParams.gyroStdDev(i), dvlSimParamsReturned.gyroStdDev(i),
                1e-12);
  }
  EXPECT_NEAR(dvlSimParams.resolution, dvlSimParamsReturned.resolution, 1e-12);
  EXPECT_NEAR(dvlSimParams.beamAngle, dvlSimParamsReturned.beamAngle, 1e-12);
  EXPECT_NEAR(dvlSimParams.minAltitude, dvlSimParamsReturned.minAltitude,
              1e-12);
  EXPECT_NEAR(dvlSimParams.maxAltitude, dvlSimParamsReturned.maxAltitude,
              1e-12);
  EXPECT_NEAR(dvlSimParams.downDistanceToGround,
              dvlSimParamsReturned.downDistanceToGround, 1e-12);
  EXPECT_NEAR(dvlSimParams.maxVelocity, dvlSimParamsReturned.maxVelocity,
              1e-12);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(dvlSimParams.p_bs_b(i), dvlSimParamsReturned.p_bs_b(i), 1e-12);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(dvlSimParams.C_s_b(i, j), dvlSimParamsReturned.C_s_b(i, j),
                  1e-12);
    }
  }
}

/**
 * @brief Test that the initial scale factor is in the defined interval.
*/
TEST_F(DvlSimulatorTest, ScaleFactorInIntervalTest) {
  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator->getDvlSimParams();

  // Set the DVL simulation parameters
  dvlSimParams.intervalInitialScaleFactor = 0.75;  // (%)
  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Test for 1000 random scale factors
  for (int i = 0; i < 1000; i++) {
    pDvlSimulator->resetSimulator();
    // Get the DVL scale factor
    double scaleFactorReturned = pDvlSimulator->getScaleFactor();
    // Compare expected with returned DVL scale factor
    EXPECT_GE(scaleFactorReturned, -0.75);
    EXPECT_LE(scaleFactorReturned, 0.75);
  }
}

/**
 * @brief Test that the initial beam bias is in the defined interval.
*/
TEST_F(DvlSimulatorTest, BeamBiasInIntervalTest) {
  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator->getDvlSimParams();

  // Set the DVL simulation parameters
  dvlSimParams.intervalInitialBeamBias =
      Eigen::Vector4d(1.0, 2.0, 3.0, 4.0);  // (m/s)
  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Test for 1000 random beam biases
  for (int i = 0; i < 1000; i++) {
    pDvlSimulator->resetSimulator();
    // Get the DVL beam bias
    Eigen::Vector4d beamBiasReturned = pDvlSimulator->getBeamBias();
    // Compare expected with returned DVL beam bias
    EXPECT_GE(beamBiasReturned(0), -1.0);
    EXPECT_GE(beamBiasReturned(1), -2.0);
    EXPECT_GE(beamBiasReturned(2), -3.0);
    EXPECT_GE(beamBiasReturned(3), -4.0);

    EXPECT_LE(beamBiasReturned(0), 1.0);
    EXPECT_LE(beamBiasReturned(1), 2.0);
    EXPECT_LE(beamBiasReturned(2), 3.0);
    EXPECT_LE(beamBiasReturned(3), 4.0);
  }
}

/**
 * @brief Test the getter and setter for the DVL enable settings.
*/
TEST_F(DvlSimulatorTest, DvlModelEnableSettingsGetterSetterTest) {
  // Fill the DVL model enable settings
  dvl_simulator::DvlModelEnableSettings dvlModelEnableSettings;
  dvlModelEnableSettings.enableBeamVelocityNoise = true;
  dvlModelEnableSettings.enableScaleFactor = true;
  dvlModelEnableSettings.enableBeamBias = true;
  dvlModelEnableSettings.enableGyroNoise = true;
  dvlModelEnableSettings.enableQuantization = true;
  dvlModelEnableSettings.enableAltitudeLimit = true;

  // Set the DVL model enable settings
  pDvlSimulator->setDvlModelEnableSettings(dvlModelEnableSettings);

  // Get the DVL model enable settings
  dvl_simulator::DvlModelEnableSettings dvlModelEnableSettingsReturned =
      pDvlSimulator->getDvlModelEnableSettings();

  // Compare expected with returned DVL model enable settings
  EXPECT_EQ(dvlModelEnableSettings.enableBeamVelocityNoise,
            dvlModelEnableSettingsReturned.enableBeamVelocityNoise);
  EXPECT_EQ(dvlModelEnableSettings.enableScaleFactor,
            dvlModelEnableSettingsReturned.enableScaleFactor);
  EXPECT_EQ(dvlModelEnableSettings.enableBeamBias,
            dvlModelEnableSettingsReturned.enableBeamBias);
  EXPECT_EQ(dvlModelEnableSettings.enableGyroNoise,
            dvlModelEnableSettingsReturned.enableGyroNoise);
  EXPECT_EQ(dvlModelEnableSettings.enableQuantization,
            dvlModelEnableSettingsReturned.enableQuantization);
  EXPECT_EQ(dvlModelEnableSettings.enableAltitudeLimit,
            dvlModelEnableSettingsReturned.enableAltitudeLimit);

  // Set the DVL model enable settings
  dvlModelEnableSettings.enableBeamVelocityNoise = false;
  dvlModelEnableSettings.enableScaleFactor = false;
  dvlModelEnableSettings.enableBeamBias = false;
  dvlModelEnableSettings.enableGyroNoise = false;
  dvlModelEnableSettings.enableQuantization = false;
  dvlModelEnableSettings.enableAltitudeLimit = false;

  // Set the DVL model enable settings
  pDvlSimulator->setDvlModelEnableSettings(dvlModelEnableSettings);

  // Get the DVL model enable settings
  dvlModelEnableSettingsReturned = pDvlSimulator->getDvlModelEnableSettings();

  // Compare expected with returned DVL model enable settings
  EXPECT_EQ(dvlModelEnableSettings.enableBeamVelocityNoise,
            dvlModelEnableSettingsReturned.enableBeamVelocityNoise);
  EXPECT_EQ(dvlModelEnableSettings.enableScaleFactor,
            dvlModelEnableSettingsReturned.enableScaleFactor);
  EXPECT_EQ(dvlModelEnableSettings.enableBeamBias,
            dvlModelEnableSettingsReturned.enableBeamBias);
  EXPECT_EQ(dvlModelEnableSettings.enableGyroNoise,
            dvlModelEnableSettingsReturned.enableGyroNoise);
  EXPECT_EQ(dvlModelEnableSettings.enableQuantization,
            dvlModelEnableSettingsReturned.enableQuantization);
  EXPECT_EQ(dvlModelEnableSettings.enableAltitudeLimit,
            dvlModelEnableSettingsReturned.enableAltitudeLimit);
}

/**
 * @brief Test the getter and setter for sample time.
*/
TEST_F(DvlSimulatorTest, SampleTimeGetterSetterTest) {
  // Set the sample time
  pDvlSimulator->setSampleTime(0.5);

  // Get the sample time
  double sampleTimeReturned = pDvlSimulator->getSampleTime();

  // Compare expected with returned sample time
  EXPECT_NEAR(sampleTimeReturned, 0.5, 1e-12);
}

/**
 * @brief Test the getter and setter for the seed.
*/
TEST_F(DvlSimulatorTest, SeedGetterSetterTest) {
  // Set the seed
  pDvlSimulator->setSeed(45);

  // Get the seed
  unsigned int seedReturned = pDvlSimulator->getSeed();

  // Compare expected with returned seed
  EXPECT_EQ(seedReturned, 45);
}

/**
 * @brief Test if the Janus geometry matrix is calculated correctly.
*/
TEST_F(DvlSimulatorTest, JanusGeometryMatrixTest) {
  // Janus geometry matrix from MATLAB calculation
  Eigen::Matrix<double, 3, 4> janusGeometryMatrixExpected;
  janusGeometryMatrixExpected << 0.270598050073099, -0.270598050073098,
      -0.270598050073099, 0.270598050073098, 0.270598050073098,
      0.270598050073099, -0.270598050073098, -0.270598050073099,
      0.923879532511287, 0.923879532511287, 0.923879532511287,
      0.923879532511287;

  // Get the calculated Janus geometry matrix from DVL simulator
  Eigen::Matrix<double, 3, 4> janusGeometryMatrix =
      pDvlSimulator->getJanusGeometryMatrix();

  // Compare expected with calculated Janus geometry matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      EXPECT_NEAR(janusGeometryMatrixExpected(i, j), janusGeometryMatrix(i, j),
                  1e-12);
    }
  }
}

/**
 * @brief Test if the Janus geometry matrix pseudoinverse is calculated correctly.
*/
TEST_F(DvlSimulatorTest, JanusGeometryMatrixPInvTest) {
  // Janus geometry matrix pseudoinverse from MATLAB calculation
  Eigen::Matrix<double, 3, 4> janusGeometryMatrixPInvExpected;
  janusGeometryMatrixPInvExpected << 0.923879532511287, -0.923879532511287,
      -0.923879532511287, 0.923879532511287, 0.923879532511287,
      0.923879532511287, -0.923879532511286, -0.923879532511287,
      0.270598050073098, 0.270598050073098, 0.270598050073099,
      0.270598050073099;

  // Get the calculated Janus geometry matrix pseudoinverse from DVL simulator
  Eigen::Matrix<double, 3, 4> janusGeometryMatrixPInv =
      pDvlSimulator->getJanusGeometryMatrixPInv();

  // Compare expected with calculated Janus geometry matrix pseudoinverse
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      EXPECT_NEAR(janusGeometryMatrixPInvExpected(i, j),
                  janusGeometryMatrixPInv(i, j), 1e-12);
    }
  }
}

/**
 * @brief Test if the DVL measurement generation function works correctly.
*/
TEST_F(DvlSimulatorTest, MeasurementGenerationTest) {
  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator->getDvlSimParams();

  // Set the DVL simulation parameters
  dvlSimParams.biasStdDev = 1e-03;                       // (m/s/sqrt(s))
  dvlSimParams.p_bs_b = Eigen::Vector3d(1.0, 2.0, 3.0);  // (m)
  dvlSimParams.C_s_b << 0, 1, 0, -1, 0, 0, 0, 0, -1;     // (-)

  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Fill the DVL model enable settings
  dvl_simulator::DvlModelEnableSettings dvlModelEnableSettings;
  dvlModelEnableSettings.enableBeamVelocityNoise = true;
  dvlModelEnableSettings.enableScaleFactor = true;
  dvlModelEnableSettings.enableBeamBias = true;
  dvlModelEnableSettings.enableGyroNoise = true;
  dvlModelEnableSettings.enableQuantization = false;
  dvlModelEnableSettings.enableAltitudeLimit = false;
  dvlModelEnableSettings.enableVelocityLimit = false;

  // Set the DVL model enable settings
  pDvlSimulator->setDvlModelEnableSettings(dvlModelEnableSettings);

  // Set the random numbers to fixed values in the DVL simulator
  pDvlSimulator->setUseFixedRandomNumbers(true);

  // Reset the simulator to ensure that fixed random numbers are used
  pDvlSimulator->resetSimulator();

  // Generate a DVL measurement
  Eigen::Vector3d p_nb_n = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Vector3d v_nb_b = Eigen::Vector3d(4.0, 5.0, 6.0);
  Eigen::Vector3d w_nb_b = Eigen::Vector3d(7.0, 8.0, 9.0);

  dvl_simulator::DvlMeasurement dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  Eigen::Vector3d velocity = dvlMeasurement.velocity;
  Eigen::Matrix3d covariance = dvlMeasurement.covariance;
  Eigen::Vector4d beamVelocities = dvlMeasurement.beamVelocities;
  double scaleFactor = pDvlSimulator->getScaleFactor();
  Eigen::Vector4d beamBias = pDvlSimulator->getBeamBias();
  bool velocityValid = dvlMeasurement.velocityValid;

  // Define MATLAB model output expected values
  Eigen::Vector3d velocityExpected =
      Eigen::Vector3d(34.425, 18.3099249493586, -28.6832988943165);
  Eigen::Matrix3d covarianceExpected;
  covarianceExpected << 0.000344794018391332, -3.53225885186072e-07,
      -8.8873679455023e-07, -3.53225885186112e-07, 0.00034353512433455,
      -1.29143676976532e-06, -8.8873679455023e-07, -1.29143676976532e-06,
      3.04465253260389e-05;
  Eigen::Vector4d beamVelocitiesExpected =
      Eigen::Vector4d(-12.355444911315, -30.7351206588478, -40.8953806354103,
                      -22.0137048878775);
  double scaleFactorExpected = 1.25;
  Eigen::Vector4d beamBiasExpected =
      Eigen::Vector4d(0.249981167540404, 0.49995488193495, 0.749928596329497,
                      0.999902310724043);
  bool velocityValidExpected = true;

  // Compare velocity expected with calculated DVL measurement
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(velocityExpected(i), velocity(i), 1e-12);
  }

  // Compare scale factor expected with calculated scale factor
  EXPECT_NEAR(scaleFactorExpected, scaleFactor, 1e-12);

  // Compare beam bias expected with calculated beam bias
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(beamBiasExpected(i), beamBias(i), 1e-12);
  }

  // Compare covariance expected with calculated covariance
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(covarianceExpected(i, j), covariance(i, j), 1e-12);
    }
  }

  // Compare beam velocities expected with calculated beam velocities
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(beamVelocitiesExpected(i), beamVelocities(i), 1e-12);
  }

  // Compare velocity valid expected with simulator velocity valid
  EXPECT_EQ(velocityValidExpected, velocityValid);
}

/**
 * @brief Test if the DVL measurement is quantized to the configured resolution.
*/
TEST_F(DvlSimulatorTest, QuantizationTest) {
  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator->getDvlSimParams();

  // Set the DVL simulation parameters
  dvlSimParams.resolution = 0.275;  // (m/s)

  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Set the DVL model enable settings
  pDvlSimulator->setEnableQuantization(true);

  // Generate a DVL measurement
  Eigen::Vector3d p_nb_n = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d v_nb_b = Eigen::Vector3d(0.1, 0.45, 0.75);
  Eigen::Vector3d w_nb_b = Eigen::Vector3d(0.0, 0.0, 0.0);

  dvl_simulator::DvlMeasurement dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  Eigen::Vector3d velocity = dvlMeasurement.velocity;

  // Define expected quantized velocity values
  Eigen::Vector3d velocityExpected = Eigen::Vector3d(0.0, 0.55, 0.825);

  // Compare beam velocities expected with calculated beam velocities
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(velocityExpected(i), velocity(i), 1e-12);
  }
}

/**
 * @brief Test if the atltitude limit is enforced correctly.
*/
TEST_F(DvlSimulatorTest, AltitudeLimitTest) {
  // Get the DVL simulation parameters
  dvl_simulator::DvlSimParams dvlSimParams = pDvlSimulator->getDvlSimParams();

  // Set the DVL simulation parameters
  dvlSimParams.minAltitude = 0.5;            // (m)
  dvlSimParams.maxAltitude = 10.0;           // (m)
  dvlSimParams.downDistanceToGround = -5.0;  // (m)

  pDvlSimulator->setDvlSimParams(dvlSimParams);

  // Set the DVL model enable settings
  pDvlSimulator->setEnableAltitudeLimit(true);

  // Generate a DVL measurement
  Eigen::Vector3d p_nb_n = Eigen::Vector3d(0.0, 0.0, 5.0);
  Eigen::Vector3d v_nb_b = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d w_nb_b = Eigen::Vector3d(0.0, 0.0, 0.0);

  dvl_simulator::DvlMeasurement dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  bool velocityValid = dvlMeasurement.velocityValid;
  bool altitudeLimitViolated = pDvlSimulator->getAltitudeLimitViolated();

  // Compare expected and simulated validity flags
  EXPECT_EQ(true, velocityValid);
  EXPECT_EQ(false, altitudeLimitViolated);

  // Set new down position
  p_nb_n = Eigen::Vector3d(0.0, 0.0, 6.0);

  dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  velocityValid = dvlMeasurement.velocityValid;
  altitudeLimitViolated = pDvlSimulator->getAltitudeLimitViolated();

  // Compare expected and simulated validity flags
  EXPECT_EQ(false, velocityValid);
  EXPECT_EQ(true, altitudeLimitViolated);

  // Set new down position
  p_nb_n = Eigen::Vector3d(0.0, 0.0, -5.1);

  dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  velocityValid = dvlMeasurement.velocityValid;
  altitudeLimitViolated = pDvlSimulator->getAltitudeLimitViolated();

  // Compare expected and simulated validity flags
  EXPECT_EQ(false, velocityValid);
  EXPECT_EQ(true, altitudeLimitViolated);

  // Set new down position
  p_nb_n = Eigen::Vector3d(0.0, 0.0, -6.0);

  dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  velocityValid = dvlMeasurement.velocityValid;
  altitudeLimitViolated = pDvlSimulator->getAltitudeLimitViolated();

  // Compare expected and simulated validity flags
  EXPECT_EQ(false, velocityValid);
  EXPECT_EQ(true, altitudeLimitViolated);
}

/**
 * @brief Test if the velocity limit is enforced correctly.
*/
TEST_F(DvlSimulatorTest, VelocityLimitTest) {
  // Set the DVL model enable settings
  pDvlSimulator->setEnableVelocityLimit(true);

  // Generate a DVL measurement
  Eigen::Vector3d p_nb_n = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d v_nb_b = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Vector3d w_nb_b = Eigen::Vector3d(0.0, 0.0, 0.0);

  dvl_simulator::DvlMeasurement dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  bool velocityValid = dvlMeasurement.velocityValid;

  // Compare velocity valid expected with simulator velocity valid
  EXPECT_EQ(true, velocityValid);

  // Get the velocity limit violated flag
  bool velocityLimitViolated = pDvlSimulator->getVelocityLimitViolated();

  // Compare velocity limit violated expected with simulator velocity limit
  EXPECT_EQ(false, velocityLimitViolated);

  // Set new velocity
  v_nb_b = Eigen::Vector3d(0.1, 0.2, 0.3);

  dvlMeasurement =
      pDvlSimulator->generateDvlMeasurement(p_nb_n, v_nb_b, w_nb_b);

  velocityValid = dvlMeasurement.velocityValid;

  // Compare velocity valid expected with simulator velocity valid
  EXPECT_EQ(true, velocityValid);

  // Get the velocity limit violated flag
  velocityLimitViolated = pDvlSimulator->getVelocityLimitViolated();

  // Compare velocity limit violated expected with simulator velocity limit
  EXPECT_EQ(false, velocityLimitViolated);
}
