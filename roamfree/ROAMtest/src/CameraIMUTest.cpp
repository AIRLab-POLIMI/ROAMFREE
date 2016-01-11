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
 * CameraIMUTest.cpp
 *
 *  Created on: Jan 6, 2016
 *      Author: davide
 *
 */

#include <random>
#include <iostream>

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMestimation/StochasticProcessFactory.h"
#include "ROAMimu/IMUIntegralHandler.h"

#include "ROAMvision/EuclideanFeatureHandler.h"

using namespace std;
using namespace ROAMestimation;
using namespace ROAMimu;
using namespace ROAMvision;

void add_gaussian_noise(double *to, unsigned int size, double mean,
    double std) {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::normal_distribution<double> dist(mean, std);

  for (int k = 0; k < size; ++k) {
    to[k] += dist(gen);
  }
}

const static int _OFF = -1;

int main(int argc, char *argv[]) {

  double imuRate = 100; // Hz rate of the IMU readings
  double poseRate = 10; // Hz rate at which pose vertices have to be maintained
  int gpsDivisor = 10; // how many IMU constraint (hndl.step(...) == true) for each GPS?

  // parameters for the generation of deterministic errors on the IMU readings

  double ba_x = 0.0; // biases on the accelerometer measurements
  double ba_y = 0.0;
  double ba_z = 0.0;

  double bw_x = 0.0; // biases on the gyroscope measurements
  double bw_y = 0.0;
  double bw_z = 0.0;

  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  ret = system("rm /tmp/roamfree/*");

  f->setLowLevelLogging(true); // default log folder

  /* ---------------------- Configure sensors ---------------------- */

  // -- accelerometer bias
  Eigen::VectorXd accBias0(3);  // initial value
  accBias0 << 0.0, 0.0, 0.0;

  Eigen::MatrixXd randomWalkNoiseVar = 10 * Eigen::MatrixXd::Identity(3, 3);

  ParameterWrapper_Ptr ba_par =
      StochasticProcessFactory::addEucl3DRandomConstant(f,
          "IMUintegralDeltaP_Ba", accBias0);

  // -- gyroscope bias
  Eigen::VectorXd gyroBias0(3); // initial value
  gyroBias0 << 0.0, 0.0, 0.0;

  ParameterWrapper_Ptr bw_par =
      StochasticProcessFactory::addEucl3DRandomConstant(f,
          "IMUintegralDeltaP_Bw", gyroBias0);

  bw_par->setFixed(true);
  ba_par->setFixed(true);

  Eigen::VectorXd T_OS_IMU(7); // Transformation between IMU and robot frame
  T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // -- IMU integral handler construction
  IMUIntegralHandler hndl(f, "IMUintegral", round(imuRate / poseRate),
      1.0 / imuRate, ba_par, bw_par, T_OS_IMU);

  // -- white noise on accelerometer and gyro
  Eigen::Matrix<double, 6, 6> & sensorNoises = hndl.getSensorNoises();
  sensorNoises.diagonal() << 0.0016, 0.0016, 0.0016, 1.15172e-05, 1.15172e-05, 1.15172e-05;
//  sensorNoises.diagonal() << 10.0, 10.0, 10.0, 1.0, 1.0, 1.0;

  // -- GPS Sensor
  Eigen::VectorXd T_OS_GPS(7); // Transformation between GPS and robot frame
  T_OS_GPS << 0.0, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // non master sensor, sequential sensor
  f->setSensorFrame("GPS", T_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(2.5 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3); // standalone noise
//  GPSCov = pow(0.05, 2) * Eigen::MatrixXd::Identity(3, 3); // carrier phase differential nosise

  // -- CAMERA
  Eigen::VectorXd T_OS_CAM(7); // Transformation between CAMERA and robot frame
  T_OS_CAM << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  Eigen::VectorXd cm(9); // Intrinsic camera calibration matrix
  cm << 1454.5, 0.0, 1024.0, 0.0, 1454.5, 1024.0, 0.0, 0.0, 1.0; // ideal Ximea MQ042MG 4Mp camera with 8mm lens
  double im_width = 2048, im_height = 2048;

  EuclideanFeatureHandler camera;
  camera.init(f, "Camera", T_OS_CAM, cm);

  // generate 3D features on a sphere
  Eigen::Vector3d r; // vector that is rotated to generate features
  r << 10.0, 0.0, 0.0;

  int Ny = 10, Np = 5; // number of yaw and pitch steps

  vector<Eigen::Vector3d> features(Ny * Np);

  for (int iy = 0; iy < Ny; iy++) {
    for (int ip = 0; ip < Np; ip++) {
      double yaw = iy * 2 * M_PI / Ny;
      double pitch = -(M_PI / 2.0 - M_PI/16) + ip * (M_PI-M_PI/8) / Np;

      Eigen::Matrix3d R;
      R << cos(pitch) * cos(yaw), -sin(yaw), cos(yaw) * sin(pitch), cos(pitch)
          * sin(yaw), cos(yaw), sin(pitch) * sin(yaw), -sin(pitch), 0, cos(
          pitch);

      features[iy * Np + ip] = R * r;
    }
  }

  Eigen::MatrixXd CAMCov = Eigen::MatrixXd::Identity(2, 2);

  /* ---------------------- Initialize ---------------------- */

  Eigen::VectorXd x0(7);

  {
    double t = 0.0;
    Eigen::VectorXd &x = x0;

#   include "../generated/Otto_GT.cppready"
  }

  hndl.init(true, 0.0, x0);

  /* ---------------------- Main loop ---------------------- */

  int cntGps = 0, cntImu = 0;
  double t = 0.0;

  bool keepOn = true;
  int cnt = 0;

  while (t <= 600.0) {

    // generate synthetic accelerometer and gyroscope reading

    Eigen::VectorXd za(3), zw(3);

    {
#     include "../generated/Otto_w.cppready"

      zw(0) += bw_x;
      zw(1) += bw_y;
      zw(2) += bw_z;
    }

    {
#     include "../generated/Otto_a.cppready"

      za(0) += ba_x;
      za(1) += ba_y;
      za(2) += ba_z;
    }

    // make an integration step
    if (hndl.step(za.data(), zw.data())) { // if we have finished:

      PoseVertexWrapper_Ptr curx_ptr = f->getNewestPose();

      // add a GPS measurement
      if (cntImu % gpsDivisor == 0) {
        Eigen::VectorXd zgps(3);

        {
          double t = curx_ptr->getTimestamp(); // shadows global t

#         include "../generated/Otto_gps.cppready"
        }

        f->addMeasurement("GPS", curx_ptr->getTimestamp(), zgps, GPSCov,
            curx_ptr);
        cntGps++;
      }

      // add Camera measurement
      Eigen::VectorXd x_gt(7);
      {
        double t = curx_ptr->getTimestamp();
        Eigen::VectorXd &x = x_gt;

#       include "../generated/Otto_GT.cppready"
      }

      for (int n = 0; n < features.size(); n++) {
        Eigen::VectorXd zcam(2);
        Eigen::VectorXd testz(1);

        {
          Eigen::VectorXd &x = x_gt;
          Eigen::Vector3d &lw = features[n];

          {
#           include "../generated/ImagePlaneProjection_Zhat.cppready"
          }

          {
#           include "../generated/ImagePlaneProjection_testZ.cppready"
          }
        }

        if (zcam(0) >= 0.0 && zcam(0) <= im_width && zcam(1) >= 0
            && zcam(1) < im_height) {
          if (testz(0) >= 0) {
            camera.addFeatureObservation(n, curx_ptr->getTimestamp(), zcam,
                CAMCov);
          }
        }
      }

      // do the estimation
      cntImu++;

      if (t > 10.0 && cntImu % ((int) gpsDivisor) == 0) { // after 5s of data, then each gps

        f->getOldestPose()->setFixed(true);
        keepOn = f->estimate(10);

        if (!keepOn) {
          return 1;
        }

        cnt++;

        //f->marginalizeOldNodes(10);

      }

    }

    t += 1.0 / imuRate;
  }

  return 0;
}
