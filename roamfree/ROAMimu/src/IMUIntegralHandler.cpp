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
 * IMUIntegralHandler.cpp
 *
 *  Created on: Sep 3, 2014
 *      Author: davide
 */

#include "IMUIntegralHandler.h"

#include <cstring>

#include <list>
#include <algorithm>

using namespace ROAMestimation;

namespace ROAMimu {

IMUIntegralHandler::IMUIntegralHandler(ROAMestimation::FactorGraphFilter* f,
    const std::string& name, int N, double dt,
    ROAMestimation::ParameterWrapper_Ptr ba_par,
    ROAMestimation::ParameterWrapper_Ptr bw_par, const Eigen::VectorXd &T_OS) :
    _filter(f), _sensorNameDeltaP(name + "DeltaP"), _ba_par(ba_par), _bw_par(
        bw_par), _sensorNameDeltaQ(name + "DeltaQ"), _N(N), _dt(dt), _cnt(0), _z12(
        new Eigen::VectorXd(27)), _z01(new Eigen::VectorXd(27)), _z12Cov(
        new Eigen::MatrixXd(3, 3)), _z01Cov(new Eigen::MatrixXd(3, 3)), _zGyro(
        16), _zGyroCov(3, 3), isFirst(true), _predictorEnabled(true) {

  assert(f != NULL);

  // add the DeltaP sensor
  f->addSensor(_sensorNameDeltaP, IMUintegralDeltaP, false, false); // non master, non sequential

  // set transformation between sensor and odometric center
  if (T_OS.rows() == 0) {
    Eigen::VectorXd T_OS_null(7);
    T_OS_null << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    f->setSensorFrame(_sensorNameDeltaP, T_OS_null);
  } else {
    f->setSensorFrame(_sensorNameDeltaP, T_OS);
  }

  // add the DeltaQ sensor
  f->addSensor(_sensorNameDeltaQ, IMUintegralDeltaQ, false, false); // non master, non sequential

  f->shareSensorFrame(_sensorNameDeltaP, _sensorNameDeltaQ);

  // add bridges for the parameters
  // TODO: compleately drop the dependency on <sensorname>_Ba and <sensorname>_Bw names

  f->shareParameter(_sensorNameDeltaP + "_Bw", _sensorNameDeltaQ + "_Bw");

  // clear temporaries
  _z12->setZero();
  _z01->setZero();

  _z12Cov->setZero();
  _z01Cov->setZero();
}
IMUIntegralHandler::~IMUIntegralHandler() {
  delete _z12, _z01, _z12Cov, _z01Cov;
}

void IMUIntegralHandler::init(bool isMaster, double t0,
    const Eigen::VectorXd &x0) {

  _isMaster = isMaster;

  if (_isMaster == true) {

    // initialize the filter
    if (x0.rows() == 0) {
      Eigen::VectorXd x0_null(7);
      x0_null << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

      _x0 = _filter->setInitialPose(x0_null, t0);
    } else {
      _x0 = _filter->setInitialPose(x0, t0);
    }
  } else {
    _x0 = _filter->getNearestPoseByTimestamp(t0);

    assert(_x0);
  }
}

// TODO: have this method return the vecotr of edges that have been added
bool IMUIntegralHandler::step(double* za, double* zw) {

  static Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  // storage to keep accelerations for the backward integral
  static std::list<double *> _history;

  // storage for the initial values for the biases
  static double ba_ptr_z1[3], bw_ptr_z1[3], ba_ptr_z2[3], bw_ptr_z2[3];

  bool ret = false;

  // if it is the first step, reset the integrator
  if (_cnt == 0) {

    // here we store the bias that will be used for the next contraint

    // TODO: a little inefficient thanks to the Eigen interface
    Eigen::VectorXd ba_tmp(3), bw_tmp(3);

    double t = (isFirst ? _x0->getTimestamp() : _x1->getTimestamp());
    _ba_par->getValueAt(ba_tmp, t);
    _bw_par->getValueAt(bw_tmp, t);

    //
    memcpy(ba_ptr_z2, ba_tmp.data(), 3 * sizeof(double));
    memcpy(bw_ptr_z2, bw_tmp.data(), 3 * sizeof(double));
    //*/

    /* instead of acquiring, do the integral always with zero biase
     memset(ba_ptr_z2, 0, 3 * sizeof(double));
     memset(bw_ptr_z2, 0, 3 * sizeof(double));
     //*/

#		ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr
    << "[IMUIntegralHandler] Info: acquiring current biases estimates:"
    << std::endl;
    std::cerr << "Ba: " << ba_tmp.transpose().format(CleanFmt) << std::endl;
    std::cerr << "Bw: " << bw_tmp.transpose().format(CleanFmt) << std::endl;
#		endif

    // reset the integral with bias for z1
    _itg.reset(ba_ptr_z1, bw_ptr_z1, _sensorNoises.data());
  }

  // do one integration step
  _itg.step(za, zw, _dt);
  _cnt++;

  // push this acceleration into the list
  double *tmp = new double[6];
  memcpy(tmp, za, 3 * sizeof(double));
  memcpy(tmp + 3, zw, 3 * sizeof(double));
  _history.push_back(tmp);

  // if we have done all the steps, collect the results
  if (_cnt == _N) {
#   ifdef DEBUG_PRINT_INFO_MESSAGES
      std::cerr << "[IMUIntegralHandler] Completed integration, adding poses and low level measurements ..." << std::endl;
#   endif

    // -------------------- STEP 1: z1 = z1 + delta  ---------------------- //
    //  sum the positive part of the integral to z1

    // ----- deltaP
    Eigen::Vector3d curdP;
    _itg.getDeltaPosition(curdP.data());
    _z12->head(3) += curdP;

    // jacobian of deltaP wrt ba
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> JdPba(
        _z12->data() + 3);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> curJdPba;
    _itg.getJDeltaPositionWrtBa(curJdPba.data());
    JdPba += curJdPba;

    // jacobian of deltaP wrt bw
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> JdPbw(
        _z12->data() + 15);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> curJdPbw;
    _itg.getJDeltaPositionWrtBw(curJdPbw.data());
    JdPbw += curJdPbw;

    // ba and bw employed for integration
    _itg.getAccelerometerBias(_z12->data() + 12);
    _itg.getGyroscopeBias(_z12->data() + 24);

    // measurement covariance
    Eigen::Matrix3d dPcov;
    _itg.getDeltaPositionCovariance(dPcov.data());
    *_z12Cov += dPcov; // sum covariance

    // ----- deltaQ

    // deltaQ
    _itg.getDeltaOrientation(_zGyro.data());

    // jacobian of deltaQ wrt bw
    _itg.getJDeltaOrientationWrtBw(_zGyro.data() + 4);

    // bw employed for integration
    _itg.getGyroscopeBias(_zGyro.data() + 13);

    // measurement covariance
    _itg.getDeltaOrientationCovariance(_zGyroCov.data());

    // -------------------- STEP 2: insert z1 edge ------------------------ //

    // the z1 measurement vector is ready, insert the corresponding edge

    // at the end of the first integral, first _z12 is not ready, we have to wait
    if (isFirst == false) {

      // move the pose window ahead and add the new pose
      if (_isMaster) {
        _x2 = _filter->addPose(_x1->getTimestamp() + _dt * _N);
      } else {
        _x2 = _filter->getNearestPoseByTimestamp(
            _x1->getTimestamp() + _dt * _N);

        double dt01 = _x1->getTimestamp() - _x0->getTimestamp();
        double dt12 = _x2->getTimestamp() - _x1->getTimestamp();

        if (abs(dt01 - dt12) > 1e-6) {
          std::cerr
              << "[IMUIntegralHandler] Warning: non master, mismatch in existing poses dt of more that 1us"
              << std::endl;
        }
      }

      if (_predictorEnabled) {
        // ----- do the prediction -------------------
        // assuming v0 = 0.

        const int _OFF = -1;

        const Eigen::VectorXd & x0 = _x0->getEstimate();
        const Eigen::VectorXd & x1 = _x1->getEstimate();

        Eigen::VectorXd s2hat(7);

        const double & sO1 = _filter->getParameterByName(
            _sensorNameDeltaP + "_SOx")->getEstimate()(0);
        const double & sO2 = _filter->getParameterByName(
            _sensorNameDeltaP + "_SOy")->getEstimate()(0);
        const double & sO3 = _filter->getParameterByName(
            _sensorNameDeltaP + "_SOz")->getEstimate()(0);

        const double & qOS1 = _filter->getParameterByName(
            _sensorNameDeltaP + "_qOSx")->getEstimate()(0);
        const double & qOS2 = _filter->getParameterByName(
            _sensorNameDeltaP + "_qOSy")->getEstimate()(0);
        const double & qOS3 = _filter->getParameterByName(
            _sensorNameDeltaP + "_qOSz")->getEstimate()(0);

        const Eigen::VectorXd & imuDp = _z12->head(3);
        const Eigen::VectorXd & imuDq = _zGyro;

        const double Dt = _dt * _N;

        // this predictor assume IMUcentric platform (T_WS = T_WR)
//#			include "generated/IMUpredictor_S2.cppready"

#				include "generated/IMUpredictor_S2genericSO.cppready"

        _x2->setEstimate(s2hat); // initialize with the computed prediction
      } else {
        _x2->setEstimate(_x1->getEstimate()); // initialize with the previous pose estimate
      }

      // -------------------------------------------

      // add the DeltaP measurement
      _filter->addMeasurement(_sensorNameDeltaP, _x2->getTimestamp(), *_z12,
          *_z12Cov, _x2, _x1, _x0);

      // handle pending measurements
      _filter->handleDeferredMeasurements();

      // prepare for the next edge
      _x0 = _x1;
      _x1 = _x2;

      ret = true;
    } else {

      if (_isMaster == true) {
        _x1 = _filter->addPose(_x0->getTimestamp() + _dt * _N);
      } else {
        _x1 = _filter->getNearestPoseByTimestamp(_x0->getTimestamp() + _dt * _N);
      }

      if (_predictorEnabled) {
        // ----- do the prediction -------------------

        const int _OFF = -1;

        const Eigen::VectorXd & x0 = _x0->getEstimate();

        Eigen::VectorXd s1hat(7);

        const double & sO1 = _filter->getParameterByName(
            _sensorNameDeltaP + "_SOx")->getEstimate()(0);
        const double & sO2 = _filter->getParameterByName(
            _sensorNameDeltaP + "_SOy")->getEstimate()(0);
        const double & sO3 = _filter->getParameterByName(
            _sensorNameDeltaP + "_SOz")->getEstimate()(0);

        const double & qOS1 = _filter->getParameterByName(
            _sensorNameDeltaP + "_qOSx")->getEstimate()(0);
        const double & qOS2 = _filter->getParameterByName(
            _sensorNameDeltaP + "_qOSy")->getEstimate()(0);
        const double & qOS3 = _filter->getParameterByName(
            _sensorNameDeltaP + "_qOSz")->getEstimate()(0);

        const Eigen::VectorXd & imuDp = curdP;
        const Eigen::VectorXd & imuDq = _zGyro;

        const double Dt = _dt * _N;

        // this predictor assume IMUcentric platform (T_WS = T_WR)
//#			include "generated/IMUpredictor_S1.cppready"

#				include "generated/IMUpredictor_S1genericSO.cppready"

        _x1->setEstimate(s1hat); // initialize with the computed prediction
      } else {
        _x1->setEstimate(_x0->getEstimate()); // initialize with previous pose estimate
      }

      // -------------------------------------------

      isFirst = false;
    }

    // whatever it was the first or not, now we have to add the gyro

    // add the DeltaQ measurement
    _filter->addMeasurement(_sensorNameDeltaQ, _x1->getTimestamp(), _zGyro,
        _zGyroCov, _x1, _x0);

    // -------------------- STEP 3: backward integral --------------------- //
    // do the integral backwards with the stored measaurements

    // reset the integrator with the biases for z2

    _itg.reset(ba_ptr_z2, bw_ptr_z2, _sensorNoises.data());

    for (auto it = _history.rbegin(); it != _history.rend(); ++it) {
      double *za = *it;
      double *zw = *it + 3;

      _itg.step(za, zw, -_dt); // negative time step

      delete[] *it; // delete the storage for current measurement;
    }
    _history.clear();

    // -------------------- STEP 4: z2 = delta ---------------------------- //
    // intialize z2 with the negative part of the integral

    // ----- deltaP
    _itg.getDeltaPosition(_z01->data());

    // jacobian of deltaP wrt ba
    _itg.getJDeltaPositionWrtBa(_z01->data() + 3);

    // jacobian of deltaP wrt bw
    _itg.getJDeltaPositionWrtBw(_z01->data() + 15);

    // measurement covariance
    _itg.getDeltaPositionCovariance(_z01Cov->data());

    // -------------------- STEP 5: swap storage for z1 and z2 ------------ //

    std::swap(_z01, _z12);
    std::swap(_z01Cov, _z12Cov);
    std::swap(ba_ptr_z2, ba_ptr_z1);
    std::swap(bw_ptr_z2, bw_ptr_z1);

    // -------------------- STEP 6: reset the counter --------------------- //

    _cnt = 0;

  }

  return ret;
}

void IMUIntegralHandler::getCurrentDeltaPosition(double* x) {
  _itg.getDeltaPosition(x);
}

void IMUIntegralHandler::getCurrendDeltaOrientation(double* q) {
  _itg.getDeltaOrientation(q);
}

void IMUIntegralHandler::setPredictorEnabled(bool enable) {
  _predictorEnabled = enable;
}

} /* namespace ROAMimu */

