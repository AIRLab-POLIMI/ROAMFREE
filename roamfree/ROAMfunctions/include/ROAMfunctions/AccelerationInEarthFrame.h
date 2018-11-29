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
 * AccelerationInEarthFrame.h
 *
 *  Created on: Nov, 29 2018
 *      Author: davide
 */

#ifndef ACCELERATIONINEARTHFRAME_H_
#define ACCELERATIONINEARTHFRAME_H_

#include <string>
#include <Eigen/Dense>

namespace ROAMfunctions {

class AccelerationInEarthFrameM {
private:

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 4;

  static const unsigned int _ORDER = 2;

  static const unsigned int _ERROR_SIZE = 3;
  static const unsigned int _NOISE_SIZE = 3;
  static const unsigned int _MEASUREMENT_SIZE = 3;

  const std::string* getParamsList() {
    return _paramsNames;
  }
  const int getNParams() {
    return _nParams;
  }

  bool predict(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {

    const static int _OFF = -1;

#   include "generated/AccelerationInEarthFrame_predictor.cppready"

    return true;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> g(params[0], 3);
    Eigen::Map<Eigen::VectorXd> b(params[1], 3);
    Eigen::Map<Eigen::VectorXd> gravity(params[2], 1);
    Eigen::Map<Eigen::VectorXd> earthrate(params[3], 1);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/AccelerationInEarthFrame_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> g(params[0], 3);
    Eigen::Map<Eigen::VectorXd> b(params[1], 3);
    Eigen::Map<Eigen::VectorXd> gravity(params[2], 1);
    Eigen::Map<Eigen::VectorXd> earthrate(params[3], 1);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -1: // jacobian wrt x
    {
#     include "generated/AccelerationInEarthFrame_JErrPOSE.cppready"
      return false; // approximation: it is almost zero (but I need the position to be evaluated)
      break;
    }
    case -2: // jacobian wrt q
    {
#     include "generated/AccelerationInEarthFrame_JErrQ.cppready"
      return true;
      break;
    }
    case -3: // jacobian wrt v
    {
#     include "generated/AccelerationInEarthFrame_JErrV.cppready"
      return true;
      break;
    }
    case -5: // jacobian wrt a
    {
#     include "generated/AccelerationInEarthFrame_JErrA.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt noises
    {
      // it is the identity matrix
      // #include "generated/AccelerationInEarthFrame_JErrNoises.cppready"
      return false;
      break;
    }
    case 1: // jacobian wrt Gain
    {
#     include "generated/AccelerationInEarthFrame_JErrG.cppready"
      return true;
      break;
    }
    case 2: // jacobian wrt bias
    {
#     include "generated/AccelerationInEarthFrame_JErrB.cppready"
      return false; // it is the identity matrix
      break;
    }
    case 3: // jacobian wrt gravity
    {
      // no gravity estimation for now
      assert(false);
    }
    case 4: // jacobian wrt earthrate
    {
      // no earth rate estimation for now
      assert(false);
    }
    }

    assert(false);
    return false;
  }
};

} /* namespace ROAMfunctions */
#endif /* ACCELERATIONINEARTHFRAME_H_ */
