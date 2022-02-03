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
 * LiDARTieFeaturesM.h
 *
 *  Created on: May 28, 2021
 *      Author: davide
 */

#ifndef LIDARTIEFEATURESM_H_
#define LIDARTIEFEATURESM_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class LiDARTieFeaturesM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 0;

  static const unsigned int _ORDER = 1;

  static const unsigned int _ERROR_SIZE = 3;
  static const unsigned int _NOISE_SIZE = 3;
  static const unsigned int _MEASUREMENT_SIZE = 6;

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

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/LiDARTieFeatures_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
    case -8: // jacobian wrt DispQ
    {
#     include "generated/LiDARTieFeatures_JErrDispQ.cppready"
      return true;
      break;
    }
    case -7: // jacobian wrt Disp
    {
#     include "generated/LiDARTieFeatures_JErrDisp.cppready"
      return true;
      break;
    }
    case 0: // jacobian wrt noises
    {
      // it is the identity matrix
//#     include "generated/LiDARTieFeatures_JErrNoises.cppready"
      return false;
      break;
    }
    }

    assert(false);
    return false;
  }
};
} /* namespace ROAMfunctions */
#endif /* LIDARTIEFEATURESM_H_ */
