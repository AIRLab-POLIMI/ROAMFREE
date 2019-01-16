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

const static int _OFF = -1;

int main(int argc, char *argv[]) {

  double imuRate = 100; // Hz rate of the IMU readings
  int gpsDivisor = 20; // One GPS reading every x IMU readings

  double leverArm = 0.25; // translation from R to GPS along y;

  // parameters of the IMU biases
  double ba_x = 0.0; //biases on the accelerometer measurements
  double ba_y = 0.0;
  double ba_z = 0.0;

  double ba_dx = 0.000; // increments on the accelerometer biases in m/s^3
  double ba_dy = 0.0;
  double ba_dz = 0.0;

  double ba_fx = 0.05; // frequency of the sinusoidal bias on the accelerometer
  double ba_fy = 0.000;
  double ba_fz = 0.0000;

  double ba_Ax = 0.001; // amplitude of the sinusoidal bias on the accelerometer
  double ba_Ay = 0.0000;
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
  f->setWriteGraph(true);
  f->setWriteHessianStructure(true);

  /* ---------------------- Configure sensors ---------------------- */

  bool isbafixed = false, isbwfixed = true;

  // Accelerometer sensor
  Eigen::VectorXd R_OS_ACC(7); // Transformation between Accelerometer and robot frame
  R_OS_ACC << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("Accelerometer", LinearAcceleration, false, true); // master sensor, sequential sensor
  f->setSensorFrame("Accelerometer", R_OS_ACC);

  //gain calibration parameter
  Eigen::VectorXd accGain0(3); // Initial gain and bias calibration parameters for accelerometer
  accGain0 << 1.0, 1.0, 1.0;

  f->addConstantParameter(Euclidean3D, "Accelerometer_G", accGain0, true);

  //gravity parameter
  Eigen::VectorXd gravity(1); // Initial gain and bias calibration parameters for accelerometer
  gravity << 9.80665;

  f->addConstantParameter(Euclidean1D, "Accelerometer_Gravity", gravity, true);

  //bias calibration parameter
  Eigen::VectorXd accBias0(3);
  accBias0 << 0.0, 0.0, 0.0;

  // it is not fixed and it is time varying
  ParameterWrapper_Ptr ba_par = f->addLinearlyInterpolatedParameter(Euclidean3D,
      "Accelerometer_B", accBias0, isbafixed, 1.0);

  ba_par->setProcessModelType(RandomWalk);
  ba_par->setRandomWalkNoiseCov(10 * Eigen::MatrixXd::Identity(3, 3));

  Eigen::MatrixXd accelerometerCov(3, 3); // covariance of Accelerometer readings
  accelerometerCov = 0.0016 * Eigen::MatrixXd::Identity(3, 3);

  // Gyroscope sensor

  f->addSensor("Gyroscope", AngularVelocity, true, true);
  f->shareSensorFrame("Accelerometer", "Gyroscope"); // usually gyro and acc are housed together

  //gain calibration parameter
  Eigen::VectorXd gyroGain0(3);
  gyroGain0 << 1.0, 1.0, 1.0;

  f->addConstantParameter(Euclidean3D, "Gyroscope_G", gyroGain0, true);

  //bias calibration parameter
  Eigen::VectorXd gyroBias0(3);
  gyroBias0 << 0.0, 0.0, 0.0;

  Eigen::Matrix3d gyroBias0Cov = 1e-6 * Eigen::MatrixXd::Identity(3, 3);

  /*
   f->addConstantParameter(Euclidean3D, "Gyroscope_B", gyroBias0, true);
   f->addPriorOnConstantParameter(Euclidean3DPrior, "Gyroscope_B", gyroBias0, gyroBias0Cov);
   //*/

  //
  ParameterWrapper_Ptr bw_par = f->addLinearlyInterpolatedParameter(Euclidean3D,
      "Gyroscope_B", gyroBias0, isbwfixed, 1.0);

  bw_par->setProcessModelType(RandomWalk);
  bw_par->setRandomWalkNoiseCov(Eigen::MatrixXd::Identity(3, 3));
  //*/

  Eigen::MatrixXd gyroscopeCov(3, 3); // covariance of Gyroscope readings
  gyroscopeCov = 1.15172e-05 * Eigen::MatrixXd::Identity(3, 3);

  // GPS Sensor

  Eigen::VectorXd R_OS_GPS(7); // Transformation between Odometer and robot frame
  R_OS_GPS << 0.0, leverArm, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // master sensor, sequential sensor
  f->setSensorFrame("GPS", R_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(2.5 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3);

  /* ---------------------- Initialize ---------------------- */

  Eigen::VectorXd x0(7), x1(7);

  // compute the ground truth for the first two poses
  {
    Eigen::VectorXd &x = x0;
    double t = 0.0;

#   include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_GT.cppready"
  }
  {
    Eigen::VectorXd &x = x1;
    double t = 1 / imuRate;

#   include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_GT.cppready"
  }

  double t = 0.0;
  PoseVertexWrapper_Ptr firstPose = f->setInitialPose(x0, t);

  t += 1 / imuRate;

  // possibly put a prior on the initial value fo the gyroscope bias
  f->addPriorOnTimeVaryingParameter(Euclidean3DPrior, "Gyroscope_B", 0.0,
      gyroBias0, gyroBias0Cov);
  //*/

  /* ---------------------- Main loop ---------------------- */

  int cntGps = 0, cntImu = 0, cntEst = 0;

  bool keepOn = true;

  while (t <= 120.0) {

    Eigen::VectorXd za(3), zw(3);

    // generate synthetic accelerometer and gyroscope reading

    {
#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_w.cppready"

      zw(0) += bw_x + bw_dx * t + bw_Ax * sin(2 * M_PI * bw_fx * t);
      zw(1) += bw_y + bw_dy * t + bw_Ay * sin(2 * M_PI * bw_fy * t);
      zw(2) += bw_z + bw_dz * t + bw_Az * sin(2 * M_PI * bw_fz * t);
    }

    {
#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_a.cppready"

      za(0) += ba_x + ba_dx * t + ba_Ax * sin(2 * M_PI * ba_fx * t);
      za(1) += ba_y + ba_dy * t + ba_Ay * sin(2 * M_PI * ba_fy * t);
      za(2) += ba_z + ba_dz * t + ba_Az * sin(2 * M_PI * ba_fz * t);
    }

    f->addSequentialMeasurement("Gyroscope", t, zw, gyroscopeCov);

    f->addSequentialMeasurement("Accelerometer", t, za, accelerometerCov);
    
    
//     {
//       Eigen::VectorXd x(7);
// 
//   #   include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_GT.cppready"
//       
//       f->getNewestPose()->setEstimate(x);
//     }

    if (cntImu == 0) {
      // the first time the measurement is not added since there is just a pose
      // in the graph and a new pose is inserted (so for the next everything is OK)

      // but since the measurement is not added the predictor cannot initialize
      // this pose, so we have to do it manually

      // and fix it (or put a prior) to remove gauge freedom
      // we fix the second for no special reason, it is just that t = 0.0 for it

      /*
       PoseVertexWrapper_Ptr first = f->getNthOldestPose(0);
       first->setFixed(true);
      //*/

      /*
       PoseVertexWrapper_Ptr second = f->getNthOldestPose(1);
       second->setEstimate(x1);
       second->setFixed(true);
      //*/

      // or to put a gaussian prior on it, with:
      Eigen::MatrixXd priorCov(6, 6);
      priorCov.setZero();
      priorCov.diagonal() << pow(0.01 / 3, 2), pow(0.01 / 3, 2), pow(0.01 / 3,2), 1, 1, 1;
//       priorCov.diagonal() << 1e12, 1e12, 1e12, 1e12, 1e12, 1;

      f->addPriorOnPose(firstPose, x0, priorCov);
      //*/

    }

    if (cntImu % gpsDivisor == 0) {

      // generate synthetic gps reading

      Eigen::VectorXd zgps(3);
      {
#       include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_gps.cppready"
      }

      MeasurementEdgeWrapperVector_Ptr edges = f->addSequentialMeasurement(
          "GPS", t, zgps, GPSCov);

      // initialize the connected vertex with GPS measurement
       assert (edges->size() > 0);

       PoseVertexWrapper_Ptr pose = (*edges)[0]->getConnectedPose(0);
       assert(pose);

       Eigen::VectorXd xInit = pose->getEstimate();
       xInit.segment(0,3) << zgps;
       pose->setEstimate(xInit);
       //*/

      cntGps++;
    }

    cntImu++;

    t += 1.0 / imuRate;

    // do the estimation

    if (t > 1.0 && cntImu % ((int) imuRate) == 0) { // after 1s of data, then each time
//      f->setSolverMethod(ROAMestimation::LevenbergMarquardt);      
      keepOn = f->estimate(10);
//       return 0;

      if (!keepOn) {
        return 1;
      }

      cntEst++;

//      f->marginalizeOldNodes(10);
    }
  }

  return 0;

}

