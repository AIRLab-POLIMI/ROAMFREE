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
 * ImagePlaneProjectionM.h
 *
 *  Created on: Oct 09, 2014
 *      Author: davide
 */

#ifndef IMAGEPLANEPROJECTION_H_
#define IMAGEPLANEPROJECTION_H_

#include <Eigen/Dense>

#include <iostream>

namespace ROAMfunctions {

class ImagePlaneProjectionM {

  public:
    static const bool _usedComponents[];

    static const std::string _paramsNames[];
    static const int _nParams = 9;

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
      Eigen::Map<Eigen::Vector3d> cm(params[1]);
      Eigen::Map<Eigen::Vector4d> rd(params[2]);
      Eigen::Map<Eigen::Vector2d> td(params[3]);
      Eigen::Map<Eigen::Vector2d> sk(params[4]);

      Eigen::Map<Eigen::Vector2d> extrd(params[5]);
      Eigen::Map<Eigen::Vector3d> extrdd(params[6]);
      Eigen::Map<Eigen::Vector4d> exttd(params[7]);
      Eigen::Map<Eigen::Vector2d> extsk(params[8]);

      Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

      const static int _OFF = -1;

#   include "generated/ImagePlaneProjection_Err.cppready"

      return false;
    }

    template<typename T>
    bool errorJacobian(const Eigen::VectorXd &x, double **params,
        const Eigen::VectorXd& z, int wrt,
        Eigen::MatrixBase<T> const &const_ret) {

      Eigen::Map<Eigen::Vector3d> lw(params[0]);
      Eigen::Map<Eigen::Vector3d> cm(params[1]);
      Eigen::Map<Eigen::Vector4d> rd(params[2]);
      Eigen::Map<Eigen::Vector2d> td(params[3]);
      Eigen::Map<Eigen::Vector2d> sk(params[4]);

      Eigen::Map<Eigen::Vector2d> extrd(params[5]);
      Eigen::Map<Eigen::Vector3d> extrdd(params[6]);
      Eigen::Map<Eigen::Vector4d> exttd(params[7]);
      Eigen::Map<Eigen::Vector2d> extsk(params[8]);

      Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

      const static int _OFF = -1;

      switch (wrt) {

      case -2: // jacobian wrt to q
      {
#     include "generated/ImagePlaneProjection_JErrQ.cppready"
        return true;
        break;
      }

      case -1: // jacobian wrt pose
      {
#     include "generated/ImagePlaneProjection_JErrPose.cppready"
        return true;
        break;
      }
      case 0: // jacobian wrt to noises
      {
        // it is the identity matrix
        // #include "generated/ImagePlaneProjection_JErrNoises.cppready"
        return false;
        break;
      }
      case 1: // jacobian wrt landmark
      {
#  	include "generated/ImagePlaneProjection_JErrLw.cppready"
        return true;
        break;
      }
      case 2: // jacobian wrt camera matrix
      {
#			include "generated/ImagePlaneProjection_JErrCM.cppready"
        return true;
        break;
      }
      case 3: // jacobian wrt radial distortion
      {
#     include "generated/ImagePlaneProjection_JErrRD.cppready"
        return true;
        break;
      }
      case 4: // jacobian wrt tangential distortion
      {
#     include "generated/ImagePlaneProjection_JErrTD.cppready"
        return true;
        break;
      }
      case 5: // jacobian wrt skew
      {
#     include "generated/ImagePlaneProjection_JErrSKEW.cppready"
        return true;
        break;
      }
      case 6: // jacobian wrt extended radial distortion
      {
#			include "generated/ImagePlaneProjection_JErrExtRD.cppready"
        return true;
        break;
      }
      case 7: // jacobian wrt extended radial distortion denominator
      {
#     include "generated/ImagePlaneProjection_JErrExtRDD.cppready"
        return true;
        break;
      }
      case 8: // jacobian wrt extended tangential distortion
      {
#     include "generated/ImagePlaneProjection_JErrExtTD.cppready"
        return true;
        break;
      }
      case 9: // jacobian wrt extended skew
      {
#     include "generated/ImagePlaneProjection_JErrExtSKEW.cppready"
        return true;
        break;
      }

      }

      assert(false);
      return false;
    }
};

} /* namespace ROAMfunctions */

#endif /* IMAGEPLANEPROJECTION_H_ */
