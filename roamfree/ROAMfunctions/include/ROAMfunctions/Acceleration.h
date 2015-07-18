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
 * Acceleration.h
 *
 *  Created on: Mar 28, 2013
 *      Author: davide
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include <string>
#include <Eigen/Dense>

namespace ROAMfunctions {

class AccelerationM {
private:

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 2;

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

#   include "generated/Acceleration_predictor.cppready"

    return true;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> g(params[0], 3);
    Eigen::Map<Eigen::VectorXd> b(params[1], 3);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/Acceleration_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> g(params[0], 3);
    Eigen::Map<Eigen::VectorXd> b(params[1], 3);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -2: // jacobian wrt to q
    {
#     include "generated/Acceleration_JErrQ.cppready"
      return true;
      break;
    }
    case -5: // jacobian wrt to a
    {
#     include "generated/Acceleration_JErrA.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt to noises
    {
      // it is the identity matrix
      // #include "generated/Acceleration_JErrNoises.cppready"
      return false;
      break;
    }
    case 1: // jacobian wrt to Gain
    {
#     include "generated/Acceleration_JErrG.cppready"
      return true;
      break;
    }
    case 2: // jacobian wrt to bias
    {
#     include "generated/Acceleration_JErrB.cppready"
      return false; // it is the identity matrix
      break;
    }

    }

    assert(false);
    return false;
  }
};

} /* namespace ROAMfunctions */
#endif /* ACCELERATION_H_ */
