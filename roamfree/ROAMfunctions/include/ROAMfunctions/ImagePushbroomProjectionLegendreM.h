/*
 Copyright (c) 2013-2016 Politecnico di Milano.
 All rights reserved. This program and the accompanying materials
 are made available under the terms of the GNU Lesser Public License v3
 which accompanies this distribution, and is available at
 https://www.gnu.org/licenses/lgpl.html

 Contributors:
 Davide A. Cucci (davide.cucci@epfl.ch)
 Simon Gilgien (simon.gilgien@epfl.ch
 */

/*
 * ImagePushbroomProjectionLegendreM.h
 *
 *  Created on: May 27, 2026
 *      Author: simon
 */

#ifndef IMAGEPUSHBROOMPROJECTIONLEGENDRE_H_
#define IMAGEPUSHBROOMPROJECTIONLEGENDRE_H_

#include <Eigen/Dense>

#include <iostream>

namespace ROAMfunctions {

class ImagePushbroomProjectionLegendreM {

  public:
    static const bool _usedComponents[];

    static const std::string _paramsNames[];
    static const int _nParams = 5;

    static const unsigned int _ORDER = 0;

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

      Eigen::Map<Eigen::Vector3d> lw(params[0]);
      Eigen::Map<Eigen::Vector2d> cm(params[1]);
      Eigen::Map<Eigen::Vector4d> ad(params[2]);
      Eigen::Map<Eigen::Vector4d> nd(params[3]);
      Eigen::Map<Eigen::Matrix<double, 1, 1>> sw(params[4]);

      Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

      const static int _OFF = -1;

#   include "generated/ImagePushbroomProjectionLegendre_Err.cppready"

      return false;
    }

    template<typename T>
    bool errorJacobian(const Eigen::VectorXd &x, double **params,
        const Eigen::VectorXd& z, int wrt,
        Eigen::MatrixBase<T> const &const_ret) {

      Eigen::Map<Eigen::Vector3d> lw(params[0]);
      Eigen::Map<Eigen::Vector2d> cm(params[1]);
      Eigen::Map<Eigen::Vector4d> ad(params[2]);
      Eigen::Map<Eigen::Vector4d> nd(params[3]);
      Eigen::Map<Eigen::Matrix<double, 1, 1>> sw(params[4]);

      Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

      const static int _OFF = -1;

      switch (wrt) {

      case -2: // jacobian wrt to q
      {
#       include "generated/ImagePushbroomProjectionLegendre_JErrQ.cppready"
        return true;
        break;
      }

      case -1: // jacobian wrt pose
      {
#       include "generated/ImagePushbroomProjectionLegendre_JErrPose.cppready"
        return true;
        break;
      }
      case 0: // jacobian wrt to noises
      {
        // TODO check if it is the identity matrix
#       include "generated/ImagePushbroomProjectionLegendre_JErrNoises.cppready"
        return false;
        break;
      }
      case 1: // jacobian wrt landmark
      {
#  	include "generated/ImagePushbroomProjectionLegendre_JErrLw.cppready"
        return true;
        break;
      }
      case 2: // jacobian wrt camera matrix
      {
#	include "generated/ImagePushbroomProjectionLegendre_JErrCM.cppready"
        return true;
        break;
      }
      case 3: // jacobian wrt axial distortion
      {
#       include "generated/ImagePushbroomProjectionLegendre_JErrAD.cppready"
        return true;
        break;
      }
      case 4: // jacobian wrt normal distortion
      {
#       include "generated/ImagePushbroomProjectionLegendre_JErrND.cppready"
        return true;
        break;
      }

      }

      assert(false);
      return false;
    }
};

} /* namespace ROAMfunctions */

#endif /* IMAGEPUSHBROOMPROJECTION_H_ */
