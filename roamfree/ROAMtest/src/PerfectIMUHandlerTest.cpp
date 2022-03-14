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
 * PerfectIMUHandlerTest.cpp
 *
 *  Created on: Dec 6, 2018
 *      Author: davide
  */

#include <random>
#include <iostream>

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMestimation/StochasticProcessFactory.h"
#include "ROAMimu/IMUIntegralHandler.h"

using namespace std;
using namespace ROAMestimation;
using namespace ROAMimu;

const static int _OFF = -1;

int main(int argc, char *argv[]) {

  double imuRate = 100; // Hz rate of the IMU readings
  double poseRate = 10; // Hz rate at which pose vertices have to be maintained
  int gpsDivisor = 10; // how many IMU constraint (hndl.step(...) == true) for each GPS?
 
  double leverArm = 0.25; // translation from R to GPS along y;

  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  f->setLowLevelLogging(true); // default log folder
  f->setWriteGraph(true);

  /* ---------------------- Configure sensors ---------------------- */

  // -- accelerometer bias
  Eigen::VectorXd accBias0(3);  // initial value
  accBias0 << 0.0, 0.0, 0.0;
  
  ParameterWrapper_Ptr ba_par = f->addConstantParameter(Euclidean3D, "IMUintegralDeltaP_Ba", accBias0, true);

  // -- gyroscope bias

  Eigen::VectorXd gyroBias0(3);  // initial value
  gyroBias0 << 0.0, 0.0, 0.0;

  ParameterWrapper_Ptr bw_par = f->addConstantParameter(Euclidean3D, "IMUintegralDeltaP_Bw", gyroBias0, true);

  Eigen::VectorXd T_OS_IMU(7); // Transformation between IMU and robot frame
  T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // -- IMU integral handler construction

  IMUIntegralHandler hndl(f, "IMUintegral", round(imuRate / poseRate),
      1.0 / imuRate, ba_par, bw_par, T_OS_IMU);

  // -- white noise on accelerometer and gyro

  Eigen::Matrix<double, 6, 6> & sensorNoises = hndl.getSensorNoises();
  sensorNoises.setZero();
  sensorNoises.diagonal() << 0.0016, 0.0016, 0.0016, 1.15172e-05, 1.15172e-05, 1.15172e-05;
  sensorNoises.diagonal() * (imuRate/100);
   
  // -- GPS Sensor

  Eigen::VectorXd T_OS_GPS(7); // Transformation between GPS and robot frame
  T_OS_GPS << 0.0, leverArm, 0.0, 1.0, 0.0, 0.0, 0.0;

  f->addSensor("GPS", AbsolutePosition, false, true); // non master sensor, sequential sensor
  f->setSensorFrame("GPS", T_OS_GPS);

  Eigen::MatrixXd GPSCov(3, 3);
  GPSCov = pow(1 / 3.0, 2) * Eigen::MatrixXd::Identity(3, 3);

  /* ---------------------- Initialize ---------------------- */

  Eigen::VectorXd x0(7), x1(7);

  {
#   include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_x0.cppready"
  }

  hndl.init(true, 0.0, x0);

  Eigen::MatrixXd priorCov(6, 6);
  priorCov.setZero();
  priorCov.diagonal() << pow(1.0, 2), pow(1.0, 2), pow(1.0,2), pow(0.35,2), pow(0.35,2), pow(0.35,2); // 1m, 10deg      

  PoseVertexWrapper_Ptr firstPose = f->getOldestPose();
  f->addPriorOnPose(firstPose, x0, priorCov);

  /* ---------------------- Main loop ---------------------- */

  int cntImu = 0;
  double t = 0.0;

  bool keepOn = true;
  int cnt = 0;

  while (t <= 120.0) {

    // generate synthetic accelerometer and gyroscope reading

    Eigen::VectorXd za(3), zw(3);

    {
#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_w.cppready"
    }

    {
#     include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_a.cppready"
    }

    // make an integration step
    if (hndl.step(t, za.data(), zw.data())) { // if we have finished:   

      if (cntImu % gpsDivisor == 0) {
	// add a GPS measurement
	Eigen::VectorXd zgps(3);

	PoseVertexWrapper_Ptr x1 = f->getNewestPose();

	{
	  double t = x1->getTimestamp(); // shadows global t

  #       include "../generated/AnalyticTraj_UniformlyAcceleratedCircularMotion_gps.cppready"
	}

	f->addMeasurement("GPS", x1->getTimestamp(), zgps, GPSCov, x1);
      }
      
      // do the estimation

      if (t > 19.0 && cntImu % (5*gpsDivisor) == 0) { // after 5s of data, then each gps
        keepOn = f->estimate(10);

        if (!keepOn) {
          return 1;
        }

        cnt++;

        //f->marginalizeOldNodes(10);
      }
      cntImu++;
    }    
    
    t += 1.0 / imuRate;
  }

  return 0;
}
