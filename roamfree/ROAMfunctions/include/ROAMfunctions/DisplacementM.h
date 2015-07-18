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
 * DisplacementM.h
 *
 *  Created on: Apr 10, 2014
 *      Author: davide
 */

#ifndef DISPLACEMENT_H_
#define DISPLACEMENT_H_

#include <Eigen/Dense>

#include <iostream>

namespace ROAMfunctions {

class DisplacementM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 0;

  static const unsigned int _ORDER = 1;

  static const unsigned int _ERROR_SIZE = 6;
  static const unsigned int _NOISE_SIZE = 6;
  static const unsigned int _MEASUREMENT_SIZE = 7;

  const std::string* getParamsList() {
    return _paramsNames;
  }
  const int getNParams() {
    return _nParams;
  }

  bool predict(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {

    const static int _OFF = -1;

#   include "generated/Displacement_predictor.cppready"

    return true;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    static Eigen::VectorXd w(1);
    double sign;

#		include "generated/Displacement_TestW.cppready"

		if (w(0) < 0) {
			sign = -1.0;
		} else {
			sign  = 1.0;
		}

#   include "generated/Displacement_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    static Eigen::VectorXd w(1);
    double sign;

#		include "generated/Displacement_TestW.cppready"

		if (w(0) < 0) {
			sign = -1.0;
		} else {
			sign  = 1.0;
		}

    switch (wrt) {
    case -7: // jacobian wrt to disp
    {
#     include "generated/Displacement_JErrDisp.cppready"
      return false; // "it is useless to evaluate me again"
      break;
    }

    case -8: // jacobian wrt to dispQ
    {
#     include "generated/Displacement_JErrDispQ.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt to noises
    {
      #include "generated/Displacement_JErrNoises.cppready"
      return true;
      break;
    }

    }

    assert(false);
    return false;
  }
};

} /* namespace ROAMfunctions */
#endif /* DISPLACEMENT_H_ */
