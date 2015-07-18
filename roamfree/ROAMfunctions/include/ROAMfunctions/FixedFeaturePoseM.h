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
 * FixedFeaturePositionM.h
 *
 *  Created on: Feb 12, 2014
 *      Author: davide
 */

#ifndef FIXEDFEATUREPOSE_H_
#define FIXEDFEATUREPOSE_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class FixedFeaturePoseM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 2;

  static const unsigned int _ORDER = 0;

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
    return false;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> fpos(params[0], 3);
    Eigen::Map<Eigen::VectorXd> fq(params[1], 4);

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    double sign;

    {
      Eigen::Matrix<double, 1, 1> w; // a scalar!

#   include "generated/FixedFeaturePose_TestW.cppready"

      if (w(0) < 0) {
        sign = -1.0;
      } else {
        sign = 1.0;
      }
    }

#   include "generated/FixedFeaturePose_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::VectorXd> fpos(params[0], 3);
    Eigen::Map<Eigen::VectorXd> fq(params[1], 4);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    double sign;

    {
      Eigen::Matrix<double, 1, 1> w; // a scalar!

#   include "generated/FixedFeaturePose_TestW.cppready"

      if (w(0) < 0) {
        sign = -1.0;
      } else {
        sign = 1.0;
      }
    }

    switch (wrt) {
    case -2: // jacobian wrt to q
    {
#     include "generated/FixedFeaturePose_JErrQ.cppready"
      return true;
      break;
    }
    case -1: // jacobian wrt to pose
    {
#     include "generated/FixedFeaturePose_JErrPOSE.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt to noises
    {
      // it is the identity matrix
//#     include "generated/FixedFeaturePose_JErrNoises.cppready"
      return false;
      break;
    }
    case 1: // jacobian wrt to feature position
    {
#     include "generated/FixedFeaturePose_JErrFPOS.cppready"
      return true;
      break;
    }
    case 2: // jacobian wrt to feature position
    {
#     include "generated/FixedFeaturePose_JErrFQ.cppready"
      return true;
      break;
    }

    }

    assert(false);
    return false;
  }
};
} /* namespace ROAMfunctions */
#endif /* FIXEDFEATUREPOSE_H_ */
