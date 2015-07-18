/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)    
*/

/*
 * IMUGPSFusionTest.cpp
 *
 *  Created on: Jun 18, 2015
 *      Author: davide
 */

#include <random>
#include <iostream>

#include "ROAMestimation/ROAMestimation.h"

using namespace std;
using namespace ROAMestimation;

void add_gaussian_noise(double *to, unsigned int size, double mean,
    double std) {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::normal_distribution<double> dist(mean, std);

  for (int k = 0; k < size; ++k) {
    to[k] += dist(gen);
  }
}

int main(int argc, char *argv[]) {

  /* syntetic measurement parameters
   *
   * in this test accelerometer and gyroscope readings are generated
   * for a uniformly accelerated circular motion according to the
   * following parameters (center of motion x = 0, y = 1)
   *
   */

  double imuRate = 50; // Hz rate of the IMU readings
  int gpsDivisor = 50; // One GPS reading every x IMU readings

  // motion parameters
  double r = 1.0; // meters
  double alpha = 0.01; // radians / s^2
  double w0 = 0.0; //initial angular speed
  double theta0 = -M_PI / 2.0;

  // parameters of the IMU biases
  double ba_x = 0.0; //biases on the accelerometer measurements
  double ba_y = 0.0;
  double ba_z = 0.0;

  double ba_dx = 0.0; // increments on the accelerometer biases in m/s^3
  double ba_dy = 0.0;
  double ba_dz = 0.0;

  double bw_x = 0.0; // biases on the gyroscope measurements
  double bw_y = 0.0;
  double bw_z = 0.0;

  double bw_dx = 0.0; // increments on the gyroscope biases
  double bw_dy = 0.0;
  double bw_dz = 0.0;

  double ba_fx = 0.1; // frequency of the sinusoidal bias on the accelerometer
  double ba_fy = 0.05;
  double ba_fz = 0.025;

  double ba_Ax = 0.25; // amplitude of the sinusoidal bias on the accelerometer
  double ba_Ay = 0.5;
  double ba_Az = 0.75;

  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  f->setLowLevelLogging(true); // default log folder

  /* ---------------------- Configure sensors ---------------------- */

  // Accelerometer sensor
  Eigen::VectorXd R_OS_ACC(7); // Transformation between Accelerometer and robot frame
  R_OS_ACC << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("Accelerometer", LinearAcceleration, true, true); // master sensor, sequential sensor
  f->setSensorFrame("Accelerometer", R_OS_ACC);

  //gain calibration parameter
  Eigen::VectorXd accGain0(3); // Initial gain and bias calibration parameters for accelerometer
  accGain0 << 1.0, 1.0, 1.0;

  f->addConstantParameter(Euclidean3D, "Accelerometer_G", accGain0, true);

  //bias calibration parameter
  Eigen::VectorXd accBias0(3);
  accBias0 << 0.0, 0.0, 0.0;

  // it is not fixed and it is time varying
  ParameterWrapper_Ptr ba_par = f->addLinearlyInterpolatedParameter(Euclidean3D,
      "Accelerometer_B", accBias0, false, 25 / imuRate);

  ba_par->setDerivativePriorsEnabled(true);
  ba_par->setDerivativePriorNoisCov(10 * Eigen::MatrixXd::Identity(3, 3));

  Eigen::MatrixXd accelerometerCov(3, 3); // covariance of Accelerometer readings
  accelerometerCov = 0.0016 * Eigen::MatrixXd::Identity(3, 3);
  //accelerometerCov = Eigen::MatrixXd::Identity(3, 3);

  // Gyroscope sensor

  f->addSensor("Gyroscope", AngularVelocity, false, true);
  f->shareSensorFrame("Accelerometer", "Gyroscope"); // usually gyro and acc are housed together

  //gain calibration parameter
  Eigen::VectorXd gyroGain0(3);
  gyroGain0 << 1.0, 1.0, 1.0;

  f->addConstantParameter(Euclidean3D, "Gyroscope_G", gyroGain0, true);

  //bias calibration parameter
  Eigen::VectorXd gyroBias0(3);
  gyroBias0 << 0.0, 0.0, 0.0;

  f->addConstantParameter(Euclidean3D, "Gyroscope_B", gyroBias0, true);

  Eigen::MatrixXd gyroscopeCov(3, 3); // covariance of Gyroscope readings
  gyroscopeCov = 1.15172e-05 * Eigen::MatrixXd::Identity(3, 3);
  //gyroscopeCov = Eigen::MatrixXd::Identity(3, 3);

  // GPS Sensor

  Eigen::VectorXd R_OS_GPS(7); // Transformation between Odometer and robot frame
  R_OS_GPS << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // master sensor, sequential sensor
  f->setSensorFrame("GPS", R_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(2.5 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3);

  /* ---------------------- Initialize ---------------------- */

  Eigen::VectorXd x0(7);
  x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // accelerometer sensor is of order 2. so we want that when the first acc
  // reading is provided, two poses are present in the graph and properly
  // initialized.

  double t = 0.0 - 1/imuRate;
  PoseVertexWrapper_Ptr firstPose = f->setInitialPose(x0, t);

  // to remove gauge freedon one has either to fix the last pose, with:
  //firstPose->setFixed(true);

  // or to put a gaussian prior on it, with:
  Eigen::MatrixXd priorCov(6,6);
  priorCov.setZero();
  priorCov.diagonal() << 1.0, 1.0, 1.0, 1e-4, 1e-4, 1e-4;

  f->addPriorOnPose(firstPose, x0, priorCov);

  t+= 1/imuRate;

  /* ---------------------- Main loop ---------------------- */

  int cntGps = 0, cntImu = 0, cntEst = 0;

  bool keepOn = true;

  while (t <= 200.0) {

    // generate accelerometer and gyroscope readings
    double w = w0 + alpha * t;

    Eigen::VectorXd za(3), zw(3);

    za << alpha * r + ba_x + ba_dx * t + ba_Ax * sin(2 * M_PI * ba_fx * t), std::pow(
        w, 2) * r + ba_y + ba_dy * t + ba_Ay * sin(2 * M_PI * ba_fy * t), 9.80665
        + ba_z + ba_dz * t + ba_Az * sin(2 * M_PI * ba_fz * t);

    zw << 0.0 + bw_x + bw_dx * t, bw_y + bw_dy * t, w + bw_z + bw_dz * t;

    f->addSequentialMeasurement("Accelerometer", t, za, accelerometerCov);
    if (cntImu == 0) {
      // the first time the measurement is not added since there is just a pose
      // in the graph and a new pose is inserted (so for the next everything is OK)

      // but since the measurement is not added the predictor cannot initialize
      // this pose, so we have to do it manually.

      f->getNthOldestPose(1)->setEstimate(x0);
    }

    f->addSequentialMeasurement("Gyroscope", t, zw, gyroscopeCov);

    if (cntImu % gpsDivisor == 0) {

      double x1_theta_circle = theta0 + w0 * t + 0.5 * alpha * std::pow(t, 2);

      Eigen::VectorXd zgps(3);
      zgps << r * std::cos(x1_theta_circle), 1 + r * std::sin(x1_theta_circle), 0;

      f->addSequentialMeasurement("GPS", t, zgps, GPSCov);
      cntGps++;
    }

    cntImu++;

    t += 1.0 / imuRate;

    // do the estimation

    if (t > 0.5 && cntImu % 100 == 0) { // after 1s of data, then each time

      keepOn = f->estimate(25);

      if (!keepOn) {
        return 1;
      }

      cntEst++;

      //f->marginalizeOldNodes(2);
    }
  }

  return 0;

}


