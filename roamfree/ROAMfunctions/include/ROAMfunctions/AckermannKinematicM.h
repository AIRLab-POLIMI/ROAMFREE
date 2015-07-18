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
 * AckermannM.h
 *
 *  Created on: Apr 3, 2013
 *      Author: davide
 */

#ifndef ACKERMANNM_H_
#define ACKERMANNM_H_

#include <string>
#include <Eigen/Dense>
#include <iostream>

namespace ROAMfunctions {

class AckermannM {
private:

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 4;

  static const unsigned int _ORDER = 1;

  static const unsigned int _ERROR_SIZE = 6;
  static const unsigned int _NOISE_SIZE = 6;
  static const unsigned int _MEASUREMENT_SIZE = 2;

  const std::string* getParamsList() {
    return _paramsNames;
  }
  const int getNParams() {
    return _nParams;
  }

  bool predict(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {

    Eigen::Map<Eigen::VectorXd> kSpeed(params[0], 1);
    Eigen::Map<Eigen::VectorXd> kSteer(params[1], 1);
    Eigen::Map<Eigen::VectorXd> psiSteer(params[2], 1);
    Eigen::Map<Eigen::VectorXd> l(params[3], 1);

    const static int _OFF = -1;

#   include "generated/Ackermann_predictor.cppready"

    return true;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params, const Eigen::VectorXd& z,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> kSpeed(params[0], 1);
    Eigen::Map<Eigen::VectorXd> kSteer(params[1], 1);
    Eigen::Map<Eigen::VectorXd> psiSteer(params[2], 1);
    Eigen::Map<Eigen::VectorXd> l(params[3], 1);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/Ackermann_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> kSpeed(params[0], 1);
    Eigen::Map<Eigen::VectorXd> kSteer(params[1], 1);
    Eigen::Map<Eigen::VectorXd> psiSteer(params[2], 1);
    Eigen::Map<Eigen::VectorXd> l(params[3], 1);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -4: // jacobian wrt to w
    {
#     include "generated/Ackermann_JErrW.cppready"
      return true;
      break;
    }
    case -3: // jacobian wrt to v
    {
#     include "generated/Ackermann_JErrV.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt to x
    {
      // it is the identity matrix
      //#     include "generated/Ackermann_JErrNoises.cppready"
      return false;
      break;
    }
    case 1: // jacobian wrt to kSpeed
    {
#     include "generated/Ackermann_JErrKSpeed.cppready"
      return true;
      break;
    }
    case 2: // jacobian wrt to kSteer
    {
#     include "generated/Ackermann_JErrKSteer.cppready"
      return true;
      break;
    }
    case 3: // jacobian wrt to psiSteer
    {
#     include "generated/Ackermann_JErrPsiSteer.cppready"
      return true;
      break;
    }
    case 4: // jacobian wrt to L
    {
#     include "generated/Ackermann_JErrL.cppready"
      return true;
      break;
    }

    }

    assert(false);
    return false;
  }
};
} /* namespace ROAMestimation */
#endif /* ACKERMANNM_H_ */
