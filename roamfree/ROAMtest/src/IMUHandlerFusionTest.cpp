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
 * IMUHandlerFusionTest.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: davide
 *
 *
 *      this test shows how to employ IMU pre-integration, i.e., the integration
 *      of chunks of IMU readings to generate single equivalent edges in the
 *      factor graph in a similar way as discussed in
 *
 *      Lupton, Todd, and Salah Sukkarieh. "Efficient integration of inertial
 *      observations into visual SLAM without initialization." Intelligent
 *      Robots and Systems, 2009. IROS 2009. IEEE/RSJ International Conference
 *      on. IEEE, 2009. *

 *      In this test a synthetic accelerometer bias is generated and estimated
 *
 *      A PluginViewer configuration for watching the estimation results
 *      can be fund in
 *
 *      _development/Matlab/PluginViewer/configs/ROAMtest/configIMUHandlerFusionTest.m
 *
 */

#include <random>
#include <iostream>

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMimu/IMUIntegralHandler.h"

using namespace std;
using namespace ROAMestimation;
using namespace ROAMimu;

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

  double imuRate = 100; // Hz rate of the IMU readings
  double poseRate = 10; // Hz rate at which pose vertices have to be maintained
  int gpsDivisor = 10; // how many IMU constraint (hndl.step(...) == true) for each GPS?

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

  double bw_dx = 0.00; // increments on the gyroscope biases
  double bw_dy = 0.00;
  double bw_dz = 0.00;

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

  bool isbafixed = false, isbwfixed = true;

  // we have to initialize the IMUIntegralHandler
  IMUIntegralHandler hndl(round(imuRate / poseRate), 1.0 / imuRate,
      IMUIntegralHandler::DoNotConfigureParameters);

  Eigen::Matrix<double, 6, 6> & sensorNoises = hndl.getSensorNoises();
  sensorNoises.diagonal() << 0.0016, 0.0016, 0.0016, 1.15172e-05, 1.15172e-05, 1.15172e-05;

  Eigen::VectorXd accBias(3);  // Accelerometer and Gyroscope giases
  accBias << 0.0, 0.0, 0.0;
  Eigen::VectorXd gyroBias(3);
  gyroBias << 0.0, 0.0, 0.0;

  ParameterWrapper_Ptr ba_par = f->addLinearlyInterpolatedParameter(Euclidean3D,
      "IMUintegralDeltaP_Ba", accBias, isbafixed, 1.0);

  ba_par->setDerivativePriorsEnabled(true);
  ba_par->setFixed(isbafixed);
  ba_par->setDerivativePriorNoisCov(10 * Eigen::MatrixXd::Identity(3, 3));

  ParameterWrapper_Ptr bw_par = f->addLinearlyInterpolatedParameter(Euclidean3D,
      "IMUintegralDeltaP_Bw", gyroBias, isbwfixed, 1.0);

  bw_par->setDerivativePriorsEnabled(true);
  bw_par->setFixed(isbwfixed);
  bw_par->setDerivativePriorNoisCov(10 * Eigen::MatrixXd::Identity(3, 3));

  Eigen::VectorXd T_OS_IMU(7); // Transformation between Odometer and robot frame
  T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // GPS Sensor

  Eigen::VectorXd R_OS_GPS(7); // Transformation between Odometer and robot frame
  R_OS_GPS << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // master sensor, sequential sensor
  f->setSensorFrame("GPS", R_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(2.5 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3);

  // POSE Sensor

  Eigen::VectorXd R_OS_POSE(7); // Transformation between Odometer and robot frame
  R_OS_POSE << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("POSE", AbsolutePose, false, true); // master sensor, sequential sensor
  f->setSensorFrame("POSE", R_OS_POSE);

  Eigen::MatrixXd POSECov(6, 6);
  POSECov.diagonal() << 0.0001, 0.0001, 0.0001, 0.000289, 0.000289, 0.000289;

  /* ---------------------- Initialize ---------------------- */

  Eigen::VectorXd x0(7); //initial pose
  x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // gyro bias is fixed since there is no other orientation input (enable the pose sensor for that)
  hndl.init(f, "IMUintegral", T_OS_IMU, accBias, false, gyroBias, true, x0,
      0.0);

  f->getOldestPose()->setFixed(true);

  /* ---------------------- Main loop ---------------------- */

  int cntGps = 0, cntImu = 0;
  double t = 0.0;

  bool keepOn = true;
  int cnt = 0;

  while (t <= 200.0) {

    // generate accelerometer and gyroscope readings
    double w = w0 + alpha * t;

    // motion in which the x axis is always tangent to the trajectory
    double za[] = { alpha * r + ba_x + ba_dx * t + ba_Ax*sin(2*M_PI*ba_fx*t), std::pow(w, 2) * r + ba_y
        + ba_dy * t + ba_Ay*sin(2*M_PI*ba_fy*t), 9.80665 + ba_z + ba_dz * t + ba_Az*sin(2*M_PI*ba_fz*t)};
    double zw[] = { 0.0 + bw_x + bw_dx * t, bw_y + bw_dy * t, w + bw_z
        + bw_dz * t };
    //*/

    /* motion in which the robot orientation wrt world is the identity
     double theta = theta0 + w0 * t + alpha*std::pow(t,2)/2.0;
     double za[] = {
     std::cos(theta+M_PI/2.0)*alpha*r + std::cos(theta+M_PI)*std::pow(w,2)*r + ba_x + ba_dx * t,
     std::sin(theta+M_PI/2.0)*alpha*r + std::sin(theta+M_PI)*std::pow(w,2)*r + ba_y + ba_dy * t,
     9.80665 + ba_z + ba_dz * t
     };
     double zw[] = { 0.0, 0.0, 0.0 };
     //*/

    /* corrupt measurement with some gaussian noise
     add_gaussian_noise(za, 3, 0.0, std::sqrt(sensorNoises(0, 0)));
     add_gaussian_noise(zw, 3, 0.0, std::sqrt(sensorNoises(3, 3)));
     //*/

    // make an integration step
    if (hndl.step(za, zw)) { // if we have finished:

      // --- insert the edge in the graph

      /* add a pose measurement on x1

       Eigen::VectorXd z_pose(7);

       PoseVertexWrapper_Ptr x1 = f->getNearestPoseByTimestamp(
       f->getNewestPose()->getTimestamp() - 1.0 / poseRate);

       double x1_theta_circle = theta0 + w0 * x1->getTimestamp()
       + 0.5 * alpha * std::pow(x1->getTimestamp(), 2);

       z_pose << r * std::cos(x1_theta_circle), 1
       + r * std::sin(x1_theta_circle), 0, std::cos(
       (x1_theta_circle - theta0) / 2.0), 0.0, 0.0, std::sin(
       (x1_theta_circle - theta0) / 2.0);

       // corrupt measurement with some gaussian noise
       add_gaussian_noise(z_pose.data(), 3, 0.0, std::sqrt(POSECov(0, 0)));

       if (cntImu % gpsDivisor == 0) {
       f->addMeasurement("POSE", x1->getTimestamp(), z_pose, POSECov, x1);
       cntGps++;
       }
       //*/

      // add a GPS measurement
      Eigen::VectorXd z_gps(3);

      PoseVertexWrapper_Ptr x1 = f->getNearestPoseByTimestamp(
          f->getNewestPose()->getTimestamp() - 1.0 / poseRate);

      double x1_theta_circle = theta0 + w0 * x1->getTimestamp()
          + 0.5 * alpha * std::pow(x1->getTimestamp(), 2);

      z_gps << r * std::cos(x1_theta_circle), 1 + r * std::sin(x1_theta_circle), 0;

      /* corrupt measurement with some gaussian noise
       add_gaussian_noise(z_pose.data(), 3, 0.0, std::sqrt(POSECov(0, 0)));
       //*/

      if (cntImu % gpsDivisor == 0) {
        f->addMeasurement("GPS", x1->getTimestamp(), z_gps, GPSCov, x1);
        cntGps++;
      }
      //*/

      /* initialize pose with ground truth
       Eigen::VectorXd x1_pose(7);
       x1_pose << r * std::cos(x1_theta), 1 + r * std::sin(x1_theta), 0, std::cos(
       (x1_theta + M_PI / 2.0) / 2.0), 0, 0, std::sin(
       (x1_theta + M_PI / 2.0) / 2.0);
       x1->setEstimate(x1_pose);
       //*/

      // do the estimation
      cntImu++;

      if (t > 2.0) { // after 2s of data, then each time

        if (cnt == 0) {
          f->getNthOldestPose(0)->setFixed(true);
          f->getNthOldestPose(1)->setFixed(true);
        }

        keepOn = f->estimate(10);

        if (!keepOn) {
          return 1;
        }

        cnt++;

        f->marginalizeOldNodes(2);

      }

    }

    t += 1.0 / imuRate;
  }

  return 0;
}
