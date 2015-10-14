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

const static int _OFF = -1;

int main(int argc, char *argv[]) {

  double imuRate = 100; // Hz rate of the IMU readings
  double poseRate = 10; // Hz rate at which pose vertices have to be maintained
  int gpsDivisor = 5; // how many IMU constraint (hndl.step(...) == true) for each GPS?

  double leverArm = 0.25; // translation from R to GPS along y;

  // parameters of the IMU biases
  double ba_x = 0.0; //biases on the accelerometer measurements
  double ba_y = 0.0;
  double ba_z = 0.3;

  double ba_dx = 0.0; // increments on the accelerometer biases in m/s^3
  double ba_dy = 0.0;
  double ba_dz = 0.0;

  double ba_fx = 0.05; // frequency of the sinusoidal bias on the accelerometer
  double ba_fy = 0.025;
  double ba_fz = 0.0125;

  double ba_Ax = 0.100; // amplitude of the sinusoidal bias on the accelerometer
  double ba_Ay = 0.2000;
  double ba_Az = 0.0000;

  double bw_x = 0.0; // biases on the gyroscope measurements
  double bw_y = 0.0;
  double bw_z = 0.0;

  double bw_dx = 0.00; // increments on the gyroscope biases
  double bw_dy = 0.00;
  double bw_dz = 0.00;

  double bw_fx = 0.0; // frequency of the sinusoidal bias on the gyroscope
  double bw_fy = 0.00;
  double bw_fz = 0.000;

  double bw_Ax = 0.000; // amplitude of the sinusoidal bias on the gyroscope
  double bw_Ay = 0.000;
  double bw_Az = 0.000;

  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  f->setLowLevelLogging(true); // default log folder

  /* ---------------------- Configure sensors ---------------------- */

  bool isbarcfixed = false, isbagmfixed = false, isbwfixed = true;

  // accelerometer bias

  Eigen::VectorXd accBias0(3);  // Accelerometer and Gyroscope giases
  accBias0 << 0.0, 0.0, 0.0;

  ParameterWrapper_Ptr ba_par_gm = f->addLinearlyInterpolatedParameter(
      Euclidean3D, "IMUintegralDeltaP_Ba_GM", accBias0, isbagmfixed, 1.0);

  ba_par_gm->setProcessModelType(RandomWalk);
  ba_par_gm->setFixed(isbagmfixed);
  ba_par_gm->setRandomWalkNoiseCov(10 * Eigen::MatrixXd::Identity(3, 3));

  Eigen::Matrix3d accBiasGM0Cov = 1e-4 * Eigen::MatrixXd::Identity(3, 3);

  ParameterWrapper_Ptr ba_par_rc = f->addConstantParameter(Euclidean3D,
      "IMUintegralDeltaP_Ba_RC", accBias0, isbarcfixed);
  ba_par_rc->setFixed(isbarcfixed);

  ParameterWrapperVector_Ptr toblend(new ParameterWrapperVector);
  toblend->push_back(ba_par_gm);
  toblend->push_back(ba_par_rc);

  ParameterWrapper_Ptr ba_par = f->addParameterBlender(Euclidean3D,
      "IMUintegralDeltaP_Ba", toblend);

  // gyroscope bias

  Eigen::VectorXd gyroBias0(3);
  gyroBias0 << 0.0, 0.0, 0.0;

  Eigen::Matrix3d gyroBias0Cov = 1e-4 * Eigen::MatrixXd::Identity(3, 3);

  /*
   ParameterWrapper_Ptr bw_par = f->addConstantParameter(Euclidean3D,
   "IMUintegralDeltaP_Bw", gyroBias0, isbwfixed);

   f->addPriorOnConstantParameter(Euclidean3DPrior, "IMUintegralDeltaP_Bw", gyroBias0, gyroBias0Cov);
   //*/

  //
  ParameterWrapper_Ptr bw_par = f->addLinearlyInterpolatedParameter(Euclidean3D,
      "IMUintegralDeltaP_Bw", gyroBias0, isbwfixed, 1.0);

  bw_par->setProcessModelType(RandomWalk);
  bw_par->setRandomWalkNoiseCov(10 * Eigen::MatrixXd::Identity(3, 3));
  //*/

  bw_par->setFixed(isbwfixed);

  Eigen::VectorXd T_OS_IMU(7); // Transformation between Odometer and robot frame
  T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // we have to initialize the IMUIntegralHandler
  IMUIntegralHandler hndl(f, "IMUintegral", round(imuRate / poseRate),
      1.0 / imuRate, ba_par, bw_par, T_OS_IMU);

  Eigen::Matrix<double, 6, 6> & sensorNoises = hndl.getSensorNoises();
  sensorNoises.diagonal() << 0.0016, 0.0016, 0.0016, 1.15172e-05, 1.15172e-05, 1.15172e-05;

  // GPS Sensor

  Eigen::VectorXd T_OS_GPS(7); // Transformation between Odometer and robot frame
  T_OS_GPS << 0.0, leverArm, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // master sensor, sequential sensor
  f->setSensorFrame("GPS", T_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(2.5 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3);
  //GPSCov = pow(0.001 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3);

  /* ---------------------- Initialize ---------------------- */

  Eigen::VectorXd x0(7), x1(7);

  {
#   include "../generated/Otto_x0.cppready"
  }
  {
#   include "../generated/Otto_x1.cppready"
  }

  hndl.init(true, 0.0, x0);

  f->addPriorOnTimeVaryingParameter(Euclidean3DPrior, "IMUintegralDeltaP_Ba_GM",
      0, accBias0, accBiasGM0Cov);

  // put the prior in case of no lever arm
  // f->addPriorOnTimeVaryingParameter(Euclidean3DPrior, "IMUintegralDeltaP_Bw",
  //    0.0, gyroBias0, gyroBias0Cov);

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

      zw(0) += bw_x + bw_dx * t + bw_Ax * sin(2 * M_PI * bw_fx * t);
      zw(1) += bw_y + bw_dy * t + bw_Ay * sin(2 * M_PI * bw_fy * t);
      zw(2) += bw_z + bw_dz * t + bw_Az * sin(2 * M_PI * bw_fz * t);
    }

    {
#     include "../generated/Otto_a.cppready"

      za(0) += ba_x + ba_dx * t + ba_Ax * sin(2 * M_PI * ba_fx * t);
      za(1) += ba_y + ba_dy * t + ba_Ay * sin(2 * M_PI * ba_fy * t);
      za(2) += ba_z + ba_dz * t + ba_Az * sin(2 * M_PI * ba_fz * t);
    }

    // make an integration step
    if (hndl.step(za.data(), zw.data())) { // if we have finished:

      if (cntImu == 0) {
        PoseVertexWrapper_Ptr first = f->getNthOldestPose(0);
        PoseVertexWrapper_Ptr second = f->getNthOldestPose(1);

        first->setEstimate(x0);
        first->setFixed(true);

        second->setEstimate(x1);
        second->setFixed(true);

        cerr << "t0: " << first->getTimestamp() << " t1: "
            << second->getTimestamp() << endl;

      }

      if (cntImu % gpsDivisor == 0) {

        // add a GPS measurement
        Eigen::VectorXd zgps(3);

        PoseVertexWrapper_Ptr x1 = f->getNearestPoseByTimestamp(
            f->getNewestPose()->getTimestamp() - 1.0 / poseRate);

        {
          double t = x1->getTimestamp(); // shadows global t

#         include "../generated/Otto_gps.cppready"
        }

        f->addMeasurement("GPS", x1->getTimestamp(), zgps, GPSCov, x1);
        cntGps++;
      }

      // do the estimation
      cntImu++;

      if (t > 1.0 && cntImu % ((int) gpsDivisor) == 0) { // after 5s of data, then each gps

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
