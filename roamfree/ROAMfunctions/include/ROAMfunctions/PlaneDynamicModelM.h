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
 * PlaneDynamicModelM.h
 *
 *  Created on: Feb 10, 2015
 *      Author: davide
 */

#ifndef PLANEDYNAMICMODEL_H_
#define PLANEDYNAMICMODEL_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class PlaneDynamicModelM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 14;

  static const unsigned int _ORDER = 2;

  static const unsigned int _ERROR_SIZE = 6;
  static const unsigned int _NOISE_SIZE = 6;
  static const unsigned int _MEASUREMENT_SIZE = 4;

  const std::string* getParamsList() {
    return _paramsNames;
  }
  const int getNParams() {
    return _nParams;
  }

  bool predict(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {

    const static int _OFF = -1;

#   include "generated/PlaneDynamicModel_predictor.cppready"

    return true;
  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::Matrix<double, 3, 1> > airDensity(params[0]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > fThrust(params[1]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > fDrag(params[2]);
    Eigen::Map<Eigen::Matrix<double, 1, 1> > fLat(params[3]);
    Eigen::Map<Eigen::Matrix<double, 2, 1> > fLift(params[4]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > mRoll(params[5]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > mPitch(params[6]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > mYaw(params[7]);
    Eigen::Map<Eigen::Matrix<double, 1, 1> > cBar(params[8]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > cp(params[9]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > ibd(params[10]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > ibod(params[11]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > wind(params[12]);
    Eigen::Map<Eigen::Matrix<double, 1, 1> > gravity(params[13]);


    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    static Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "", "");

#   include "generated/PlaneDynamicModel_Err.cppready"

    /*

      output some debug variables

    {
      Eigen::Matrix<double, 7, 1> dbg;

#     include "generated/PlaneDynamicModel_debug.cppready"

      std::cerr << dbg.transpose().format(CleanFmt) << std::endl;
    }
    //*/

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::Map<Eigen::Matrix<double, 3, 1> > airDensity(params[0]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > fThrust(params[1]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > fDrag(params[2]);
    Eigen::Map<Eigen::Matrix<double, 1, 1> > fLat(params[3]);
    Eigen::Map<Eigen::Matrix<double, 2, 1> > fLift(params[4]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > mRoll(params[5]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > mPitch(params[6]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > mYaw(params[7]);
    Eigen::Map<Eigen::Matrix<double, 1, 1> > cBar(params[8]);
    Eigen::Map<Eigen::Matrix<double, 4, 1> > cp(params[9]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > ibd(params[10]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > ibod(params[11]);
    Eigen::Map<Eigen::Matrix<double, 3, 1> > wind(params[12]);
    Eigen::Map<Eigen::Matrix<double, 1, 1> > gravity(params[13]);

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {

    case -12: // jacobian wrt to previous omega
    {
#     include "generated/PlaneDynamicModel_JErrWprev.cppready"
      return true;
      break;
    }

    case -11: // jacobian wrt to previous v
    {
#     include "generated/PlaneDynamicModel_JErrVprev.cppready"
      return true;
      break;
    }

    case -10: // jacobian wrt to previous q
    {
#     include "generated/PlaneDynamicModel_JErrQprev.cppready"
      return true;
      break;
    }

    case -6: // jacobian wrt to alpha
    {
#     include "generated/PlaneDynamicModel_JErrAlpha.cppready"
      return true;
      break;
    }

    case -5: // jacobian wrt to a
    {
#     include "generated/PlaneDynamicModel_JErrA.cppready"
      return true;
      break;
    }

    case -4: // jacobian wrt to w
    {
#     include "generated/PlaneDynamicModel_JErrW.cppready"
      return true;
      break;
    }

    case -3: // jacobian wrt to v
    {
#     include "generated/PlaneDynamicModel_JErrV.cppready"
      return true;
      break;
    }

    case -2: // jacobian wrt to q
    {
#     include "generated/PlaneDynamicModel_JErrQ.cppready"
      return true;
      break;
    }

    case -1: // jacobian wrt pose
    {
#     include "generated/PlaneDynamicModel_JErrPose.cppready"
      return true;
      break;
    }

    case 0: // jacobian wrt to noises
    {
      // it is the identity matrix
      // #include "generated/PlaneDynamicModel_JErrNoises.cppready"
      return false;
      break;
    }

    case 1: // jacobian wrt AirDensity
    {
#  	include "generated/PlaneDynamicModel_JErrAirDensity.cppready"
      return true;
      break;
    }

    case 2: // jacobian wrt FThrust
    {
#  	include "generated/PlaneDynamicModel_JErrFThrust.cppready"
      return true;
      break;
    }
    case 3: // jacobian wrt FDrag
    {
#  	include "generated/PlaneDynamicModel_JErrFDrag.cppready"
      return true;
      break;
    }
    case 4: // jacobian wrt FLat
    {
#  	include "generated/PlaneDynamicModel_JErrFLat.cppready"
      return true;
      break;
    }
    case 5: // jacobian wrt FLift
    {
#  	include "generated/PlaneDynamicModel_JErrFLift.cppready"
      return true;
      break;
    }
    case 6: // jacobian wrt MRoll
    {
#  	include "generated/PlaneDynamicModel_JErrMRoll.cppready"
      return true;
      break;
    }
    case 7: // jacobian wrt MPitch
    {
#  	include "generated/PlaneDynamicModel_JErrMPitch.cppready"
      return true;
      break;
    }
    case 8: // jacobian wrt MYaw
    {
#  	include "generated/PlaneDynamicModel_JErrMYaw.cppready"
      return true;
      break;
    }
    case 9: // jacobian wrt CBar
    {
#  	include "generated/PlaneDynamicModel_JErrCBar.cppready"
      return true;
      break;
    }
    case 10: // jacobian wrt Cp
    {
#  	include "generated/PlaneDynamicModel_JErrCp.cppready"
      return true;
      break;
    }
    case 11: // jacobian wrt Ibd
    {
#  	include "generated/PlaneDynamicModel_JErrIbd.cppready"
      return true;
      break;
    }
    case 12: // jacobian wrt Ibod
    {
#  	include "generated/PlaneDynamicModel_JErrIbod.cppready"
      return true;
      break;
    }
    case 13: // jacobian wrt wind
    {
#   include "generated/PlaneDynamicModel_JErrWind.cppready"
      return true;
      break;
    }

    }

    assert(false);
    return false;
  }
};

} /* namespace ROAMfunctions */

#endif /* PLANEDYNAMICMODEL_H_ */
