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
 * PerfectIMUTest.cpp
 *
 *  Created on: Nov 6, 2018
 *      Author: davide
 */

#include <random>
#include <iostream>

#include "ROAMestimation/ROAMestimation.h"

using namespace std;
using namespace ROAMestimation;

const static int _OFF = -1;

int main(int argc, char *argv[]) {

  int imuRate = 345; // Hz rate of the IMU readings
  int GPSRate = 1; // One GPS reading every x IMU readings

  double leverArm = 0.25; // translation from R to GPS along y;

  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  f->setLowLevelLogging(true); // default log folder
  f->setWriteGraph(true);
  f->setWriteHessianStructure(true);

  /* ---------------------- Configure sensors ---------------------- */

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
  gravity << 9.8;

  f->addConstantParameter(Euclidean1D, "Accelerometer_Gravity", gravity, true);

  //bias calibration parameter
  Eigen::VectorXd accBias0(3);
  accBias0 << 0.0, 0.0, 0.0;

  // it is not fixed and it is time varying
  ParameterWrapper_Ptr ba_par = f->addConstantParameter(Euclidean3D, "Accelerometer_B", accBias0, true);

  Eigen::MatrixXd accelerometerCov(3, 3); // covariance of Accelerometer readings
  accelerometerCov = 0.0016 * Eigen::MatrixXd::Identity(3, 3) * (imuRate/100);

   
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

  f->addConstantParameter(Euclidean3D, "Gyroscope_B", gyroBias0, true);

  Eigen::MatrixXd gyroscopeCov(3, 3); // covariance of Gyroscope readings
  gyroscopeCov = 1.15172e-05 * Eigen::MatrixXd::Identity(3, 3) * (imuRate/100);

  // GPS Sensor

  Eigen::VectorXd R_OS_GPS(7); // Transformation between Odometer and robot frame
  R_OS_GPS << 0.0, leverArm, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // master sensor, sequential sensor
  f->setSensorFrame("GPS", R_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(1, 2) * Eigen::MatrixXd::Identity(3, 3)*0.65;
  
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

  /* ---------------------- Main loop ---------------------- */

  int cntGps = 0, cntImu = 0, cntEst = 0;

  bool keepOn = true;

  while (t <= 120.0) {

    Eigen::VectorXd za(3), zw(3);

    // generate synthetic accelerometer and gyroscope reading

    {
#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_w.cppready"
    }

    {
#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_a.cppready"
    }

    f->addSequentialMeasurement("Gyroscope", t, zw, gyroscopeCov);

    f->addSequentialMeasurement("Accelerometer", t, za, accelerometerCov);
    
    
    {
      Eigen::VectorXd x(7);      

#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_GT.cppready"
      
      f->getNewestPose()->setEstimate(x);
    }
    
    
    f->getNewestPose()->setComputeUncertainty(true);
    
    if (cntImu == 0) {
      // the first time the measurement is not added since there is just a pose
      // in the graph and a new pose is inserted (so for the next everything is OK)

      // but since the measurement is not added the predictor cannot initialize
      // this pose, so we have to do it manually

      // and fix it (or put a prior) to remove gauge freedom
      // we fix the second for no special reason, it is just that t = 0.0 for it

      /*
       PoseVertexWrapper_Ptr first = f->getNthOldestPose(0);
       first->setEstimate(x0);
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
      priorCov.diagonal() << pow(1.0, 2), pow(1.0, 2), pow(1.0,2), pow(0.35,2), pow(0.35,2), pow(0.35,2); // 1m, 10deg      

      f->addPriorOnPose(firstPose, x0, priorCov);
      //*/

    }

    if (cntImu % (imuRate/GPSRate) == 0) {

      // generate synthetic gps reading

      Eigen::VectorXd zgps(3);
      {
	
#       include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_gps.cppready"
      }

      MeasurementEdgeWrapperVector_Ptr edges = f->addSequentialMeasurement(
          "GPS", t, zgps, GPSCov);

      // initialize the connected vertex with GPS measurement
      assert (edges->size() > 0);

//       PoseVertexWrapper_Ptr pose = (*edges)[0]->getConnectedPose(0);
//       assert(pose);
//       
//       pose->setComputeUncertainty(true);
// 
//       Eigen::VectorXd xInit = pose->getEstimate();
//       xInit.segment(0,3) << zgps;
//       pose->setEstimate(xInit);
      //*/

      cntGps++;
    }

    cntImu++;

    t += 1.0 / imuRate;

    // do the estimation

//     if (t > 1.0 && cntImu % (5 * (int) imuRate) == 0) { // after 1s of data, then each time
    if (cntImu == 5*imuRate) {
//       f->setSolverMethod(ROAMestimation::LevenbergMarquardt);      
      f->setSolverMethod(ROAMestimation::GaussNewton);      
      keepOn = f->estimate(10);      
      f->writeFinalHessian();
      return 1;


      if (!keepOn) {
        return 1;
      }

      cntEst++;

//      f->marginalizeOldNodes(10);
    }
  }

  return 0;

}

