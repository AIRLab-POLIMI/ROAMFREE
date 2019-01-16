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
 * AngularVelocity.h
 *
 *  Created on: Nov, 29 2018
 *      Author: davide
 */

#ifndef ANGULARVELOCITYINEARTHFRAME_H_
#define ANGULARVELOCITYINEARTHFRAME_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class AngularVelocityInEarthFrameM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 3;

  static const unsigned int _ORDER = 1;

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
    xhat = x; // TODO: dummy predictor

    return true;

  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> g(params[0], 3);
    Eigen::Map<Eigen::VectorXd> b(params[1], 3);
    Eigen::Map<Eigen::VectorXd> earthrate(params[2], 1);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/AngularVelocityInEarthFrame_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> g(params[0], 3);
    Eigen::Map<Eigen::VectorXd> b(params[1], 3);
    Eigen::Map<Eigen::VectorXd> earthrate(params[2], 1);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -4: // jacobian wrt w
    {
#     include "generated/AngularVelocityInEarthFrame_JErrW.cppready"
      return true;
      break;
    }
    case -2: // jacobian wrt q
    {
#     include "generated/AngularVelocityInEarthFrame_JErrQ.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt noises
    {
      // it is the identity matrix
//#     include "generated/AngularVelocityInEarthFrame_JErrNoises.cppready"
      return false;
      break;
    }
    case 1: // jacobian wrt Gain
    {
#     include "generated/AngularVelocityInEarthFrame_JErrG.cppready"
      return true;
      break;
    }
    case 2: // jacobian wrt bias
    {
#     include "generated/AngularVelocityInEarthFrame_JErrB.cppready"
      return false; // it is the identity matrix
      break;
    }
    case 3: // jacobian wrt earthrate
    {
      // no earth rate estimation...
      assert(false);
      break;
    }
    }

    assert(false);
    return false;
  }
};
} /* namespace ROAMfunctions */
#endif /* ANGULARVELOCITYINEARTHFRAME_H_ */
