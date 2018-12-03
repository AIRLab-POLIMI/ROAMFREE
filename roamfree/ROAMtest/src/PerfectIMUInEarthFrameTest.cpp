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

#include <iostream>
#include <fstream>

#include "ROAMestimation/ROAMestimation.h"

using namespace std;
using namespace ROAMestimation;

bool readStampedVector(ifstream &f, int N, double &t, Eigen::VectorXd &x);

int main(int argc, char *argv[]) {

  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  f->setLowLevelLogging(true); // default log folder
  f->setWriteGraph(true);
  f->setWriteHessianStructure(true);

  /* ---------------------- Configure accelerometer ---------------- */

  Eigen::VectorXd R_OS_ACC(7); // Transformation between Accelerometer and robot frame
  R_OS_ACC << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("Accelerometer", LinearAccelerationInEarthFrame, false, true); // master sensor, sequential sensor
  f->setSensorFrame("Accelerometer", R_OS_ACC);

  //gain calibration parameter
  Eigen::VectorXd accGain0(3); 
  accGain0 << 1.0, 1.0, 1.0;

  f->addConstantParameter(Euclidean3D, "Accelerometer_G", accGain0, true);
  
  //gravity parameter
  Eigen::VectorXd gravity(1); 
  gravity << 9.80665;

  f->addConstantParameter(Euclidean1D, "Accelerometer_Gravity", gravity, true);
  
  //earthrate parameter
  Eigen::VectorXd earthrate(1);
  earthrate << 7.292115e-5;

  f->addConstantParameter(Euclidean1D, "Accelerometer_EarthRate", earthrate, true);

  //ellipsoid parameters for the gravity model
  Eigen::VectorXd ep(2);
  ep << 6378137.0, 6356752.3142;

  f->addConstantParameter(Euclidean2D, "Accelerometer_EP", ep, true);
  
  //shift in earth frame
  Eigen::VectorXd epshift(3);
  epshift << 4368089.0, 502828.0, 4605403.0;
    
  f->addConstantParameter(Euclidean3D, "Accelerometer_EPShift", epshift, true);

  //bias calibration parameter
  Eigen::VectorXd accBias0(3);
  accBias0 << 0.0, 0.0, 0.0;

  // it is fixed and it is constant (perfect IMU..)
  ParameterWrapper_Ptr ba_par = f->addConstantParameter(Euclidean3D, "Accelerometer_B", accBias0, true);

  Eigen::MatrixXd accelerometerCov(3, 3); // covariance of Accelerometer readings
  // TODO: find another way to get the IMU rate
  accelerometerCov = 0.0016 * Eigen::MatrixXd::Identity(3, 3) * 2; // scaling of the covariance based on the frequency
   
  /* ---------------------- Configure gyroscope -------------------- */

  f->addSensor("Gyroscope", AngularVelocityInEarthFrame, true, true);
  f->shareSensorFrame("Accelerometer", "Gyroscope"); // usually gyro and acc are housed together

  //gain calibration parameter
  Eigen::VectorXd gyroGain0(3);
  gyroGain0 << 1.0, 1.0, 1.0;

  f->addConstantParameter(Euclidean3D, "Gyroscope_G", gyroGain0, true);

  //earthrate calibration parameter
  f->addConstantParameter(Euclidean1D, "Gyroscope_EarthRate", earthrate, true);

  //bias calibration parameter
  Eigen::VectorXd gyroBias0(3);
  gyroBias0 << 0.0, 0.0, 0.0;

  f->addConstantParameter(Euclidean3D, "Gyroscope_B", gyroBias0, true);

  Eigen::MatrixXd gyroscopeCov(3, 3); // covariance of Gyroscope readings
  // TODO: find another way to get the IMU rate
  gyroscopeCov = 1.15172e-05 * Eigen::MatrixXd::Identity(3, 3); // * (imuRate/100);

  /* ---------------------- Configure GPS -------------------------- */

  Eigen::VectorXd R_OS_GPS(7); // Transformation between Odometer and robot frame
  R_OS_GPS << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // master sensor, sequential sensor
  f->setSensorFrame("GPS", R_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(1, 2) * Eigen::MatrixXd::Identity(3, 3)*0.65;

  /* ---------------------- Measurement files --------------- */
  
  ifstream fa("data/PerfectIMUInEarthFrame_za.txt");
  ifstream fw("data/PerfectIMUInEarthFrame_zw.txt");
  ifstream fgps("data/PerfectIMUInEarthFrame_zgps.txt");
  ifstream fgt("data/PerfectIMUInEarthFrame_GT.txt");

  bool ok;
  
  double ta, tw, tgps, tgt;
  Eigen::VectorXd za(3), zw(3), zgps(3), gt(7);
  
  ok = readStampedVector(fgps, 3, tgps, zgps);
  ok &= readStampedVector(fgt, 7, tgt, gt);
  if (!ok) {
    return 1;
  }
  
  /* ---------------------- Initialize ---------------------- */
  
  PoseVertexWrapper_Ptr firstPose = f->setInitialPose(gt, tgt);
  
  Eigen::MatrixXd priorCov(6, 6);
  priorCov.setZero();
  priorCov.diagonal() << pow(1.0, 2), pow(1.0, 2), pow(1.0,2), pow(0.35,2), pow(0.35,2), pow(0.35,2); // 1m, 10deg      

  f->addPriorOnPose(firstPose, gt, priorCov);  
   
  /* ---------------------- Main loop ---------------------- */
  
  bool keepOn = true;
  
  while (true) {
    
    ok = readStampedVector(fa,3,ta,za);
    ok &= readStampedVector(fw,3,tw,zw);
    ok &= (fabs(ta-tw)<1e-9);

    if (!ok) {
      cerr << "za and zw files ouf of sync" << endl;
      return true;
    }
    
    f->addSequentialMeasurement("Gyroscope", tw, zw, gyroscopeCov);
    f->addSequentialMeasurement("Accelerometer", ta, za, accelerometerCov);
    
    if (fabs(tgps-ta)<1e-9) {      
    
      MeasurementEdgeWrapperVector_Ptr edges = f->addSequentialMeasurement(
          "GPS", tgps, zgps, GPSCov);

      // initialize the connected vertex with GPS measurement
      assert (edges->size() > 0);

      PoseVertexWrapper_Ptr pose = (*edges)[0]->getConnectedPose(0);
      assert(pose);

      Eigen::VectorXd xInit = pose->getEstimate();
      xInit.segment(0,3) << zgps;
      pose->setEstimate(xInit);
      
      ok = readStampedVector(fgps, 3, tgps, zgps);
      if (!ok) {
	cerr << "failed to read GPS" << endl;
	return true;
      }
    }
    
    // perfect initialization
    PoseVertexWrapper_Ptr lastPose = f->getNewestPose();    
    if (fabs(tgt-lastPose->getTimestamp()) < 1e-9) {
// 	lastPose->setEstimate(gt);
	ok = readStampedVector(fgt,7,tgt,gt);
	if (!ok) {
	  cerr << "failed to read GT" << endl;
	  return true;
	}	
    }
    //
    
    
    if ( ta > 0.0 && fabs(fmod(ta,5)) < 1e-9 ) {
//       f->setSolverMethod(LevenbergMarquardt);
//       f->estimate(0);
//       return 1;
      
      keepOn = f->estimate(10);      
      f->writeFinalHessian();
//       return 1;

      if (!keepOn) {
        return 1;
      }
    }
  }

  return 0;

}

bool readStampedVector(ifstream &f, int N, double &t, Eigen::VectorXd &x) {
  if (!f.is_open() || f.eof()) {
    return false;
  }

  char delim;

  f >> t;

  for (int k = 0; k < N; k++) {
    f >> delim;
    f >> x(k);
  }

  return !f.eof();
}