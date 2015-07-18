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
 * OrientationContinuity.cpp
 *
 *  Created on: Apr 26, 2013
 *      Author: davide
 */

#include "OrientationContinuityEdge.h"

#include "ROAMmath/MatricesOperations.h"

namespace ROAMestimation {

OrientationContinuityEdge::OrientationContinuityEdge() :
    sign(1.0) {
  _measurement = Eigen::VectorXd::Zero(3);
  _noiseCov = Eigen::Matrix3d::Identity();
}

void OrientationContinuityEdge::preIteration() {
  const Eigen::VectorXd & x =
      static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & x2 =
      static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[1])->estimate();

  const Eigen::VectorXd & z = _measurement;

  static Eigen::VectorXd w(1);
  static Eigen::Matrix3d J;

#include "generated/OrientationContinuity_TestW.cppready"

  if (w(0) < 0) {
    sign = -1.0;
  } else {
    sign = 1.0;
  }

#include "generated/OrientationContinuity_JErrNoises.cppready"

  static Eigen::Matrix3d tmpCov;

  tmpCov = J * _noiseCov * J.transpose();
  ROAMmath::inv(tmpCov, _information);
}

void OrientationContinuityEdge::computeError() {
  preIteration();

  const Eigen::VectorXd & x =
      static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & x2 =
      static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[1])->estimate();

  const Eigen::VectorXd & z = _measurement;

#include "generated/OrientationContinuity_Err.cppready"
}

void OrientationContinuityEdge::linearizeOplus() {
  const Eigen::VectorXd & x =
      static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & x2 =
      static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[1])->estimate();

  const Eigen::VectorXd & z = _measurement;
  {
#include "generated/OrientationContinuity_JErrX1.cppready"
  }
  {
#include "generated/OrientationContinuity_JErrX2.cppready"
  }
}

Eigen::Matrix3d& OrientationContinuityEdge::accessNoiseCov() {
  return _noiseCov;
}

bool OrientationContinuityEdge::read(std::istream &s) {
  for (unsigned int i = 0; i < _noiseCov.rows(); i++) {
    for (unsigned int j = i; j < _noiseCov.cols(); j++) {
      s >> _noiseCov(i, j);
      _noiseCov(j, i) = _noiseCov(i, j);
    }
  }

  return true;
}

bool OrientationContinuityEdge::write(std::ostream &s) const {
  for (unsigned int i = 0; i < _noiseCov.rows(); i++) {
    for (unsigned int j = i; j < _noiseCov.cols(); j++) {
      s << std::fixed << std::setprecision(8) << _noiseCov(i, j) << " ";
    }
  }

  return s.good();
}

} /* namespace ROAMpython */
