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
 * LinearVelocityM.h
 *
 *  Created on: Jun 28, 2013
 *      Author: davide
 */

#ifndef LINEARVELOCITYM_H_
#define LINEARVELOCITYM_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class LinearVelocityM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 0;

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

    // TODO: fake predictor
    xhat = x;

    return true;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/LinearVelocity_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -3: // jacobian wrt to v
    {
#     include "generated/LinearVelocity_JErrV.cppready"
      return false;
      break;
    }
    case 0: // jacobian wrt to noises
    {
      // it is the identity matrix
      //#include "generated/LinearVelocity_JErrNoises.cppready"
      return false;
      break;
    }

    }

    assert(false);
    return false;
  }
};
} /* namespace ROAMfunctions */
#endif /* LINEARVELOCITYM_H_ */
