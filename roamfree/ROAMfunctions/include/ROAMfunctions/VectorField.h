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
 * VectorField.h
 *
 *  Created on: Apr 30, 2013
 *      Author: davide
 */

#ifndef VECTORFIELD_H_
#define VECTORFIELD_H_

#include <string>
#include <Eigen/Dense>

namespace ROAMfunctions {

class VectorFieldM {
private:

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 3;

  static const unsigned int _ORDER = 0;

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
    return false;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double ** params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> r(params[0], 9);
    Eigen::Map<Eigen::VectorXd> s(params[1], 3);
    Eigen::Map<Eigen::VectorXd> h(params[2], 3);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/VectorField_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double ** params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> r(params[0], 9);
    Eigen::Map<Eigen::VectorXd> s(params[1], 3);
    Eigen::Map<Eigen::VectorXd> h(params[2], 3);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -2: // jacobian wrt to q
    {
#     include "generated/VectorField_JErrQ.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt to noises
    {
      // it is the identity matrix
      //#include "generated/VectorField_JErrNoises.cppready"
      return false;
      break;
    }
    case 1: // jacobian wrt parameter R (matrix)
    {
#     include "generated/VectorField_JErrR.cppready"
      return true;
      break;
    }
    case 2: // jacobian wrt parameter S (bias)
    {
#     include "generated/VectorField_JErrS.cppready"
      return false;
      break;
    }

    }

    assert(false);
    return false;
  }
};

} /* namespace ROAMfunctions */
#endif /* VECTORFIELD_H_ */
