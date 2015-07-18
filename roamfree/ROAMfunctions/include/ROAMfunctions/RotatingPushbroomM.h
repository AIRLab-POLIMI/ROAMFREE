/*
 Copyright (c) 2013-2014 Politecnico di Milano.
 All rights reserved. This program and the accompanying materials
 are made available under the terms of the GNU Lesser Public License v3
 which accompanies this distribution, and is available at
 https://www.gnu.org/licenses/lgpl.html

 Contributors:
 Davide A. Cucci (cucci@elet.polimi.it)
 */

/*
 * RotatingPushbroom.h
 *
 *  Created on: Jan 16, 2015
 *      Author: davide
 */

#ifndef ROTATINGPUSHBROOMM_H_
#define ROTATINGPUSHBROOMM_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class RotatingPushbroomM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 2;

  static const unsigned int _ORDER = 1;

  static const unsigned int _ERROR_SIZE = 2;
  static const unsigned int _NOISE_SIZE = 2;
  static const unsigned int _MEASUREMENT_SIZE = 2;

  const std::string* getParamsList() {
    return _paramsNames;
  }
  const int getNParams() {
    return _nParams;
  }

  bool predict(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {
    return false;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> h(params[0], 1);

    Eigen::Map<Eigen::Vector2d> k(params[1]);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#		include "generated/RotatingPushbroom_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> h(params[0], 1);

    Eigen::Map<Eigen::Vector2d> k(params[1]);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {

    case -8: // jacobian wrt to dispQ
    {
#     include "generated/RotatingPushbroom_JErrDispQ.cppready"
      return true;
      break;
    }
    case -7: // jacobian wrt to disp
    {
#     include "generated/RotatingPushbroom_JErrDisp.cppready"
      return true;
      break;
    }
    case -2: // jacobian wrt to q
    {
#   	include "generated/RotatingPushbroom_JErrQ.cppready"
      return true;
      break;
    }
    case -1: // jacobian wrt pose    case -8: // jacobian wrt to dispQ
    {
#     include "generated/RotatingPushbroom_JErrDispQ.cppready"
      return true;
      break;
    }
    {
#   	include "generated/RotatingPushbroom_JErrPose.cppready"
      return true;
      break;
    }

    case 0: // jacobian wrt to noises
    {
      // it is the identity matrix
      // #include "generated/RectangularObject_JErrNoises.cppready"
      return false;
      break;
    }

    case 1: // jacobian wrt ff and w
    {
#  		include "generated/RotatingPushbroom_JErrH.cppready"
      return true;
      break;
    }
    }

    assert(false);
    return false;
  }
};

} /* namespace ROAMfunctions */

#endif /* ROTATINGPUSHBROOMM_H_ */
