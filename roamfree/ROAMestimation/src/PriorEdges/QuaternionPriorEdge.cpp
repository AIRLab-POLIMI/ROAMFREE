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
 * QuaternionPriorEdge.cpp
 *
 *  Created on: Mar 26, 2019
 *      Author: davide
 */

#include "PriorEdges/QuaternionPriorEdge.h"
#include "ROAMmath/MatricesOperations.h"

namespace ROAMestimation {

QuaternionPriorEdge::QuaternionPriorEdge() {
  _measurement.resize(4);
}

void QuaternionPriorEdge::afterVertexUpdate() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::QuaternionV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  static Eigen::VectorXd w(1);
  static Eigen::Matrix<double, 3, 3> J;

# include "generated/PriorEdges/QuaternionPrior_TestW.cppready"

  if (w(0) < 0) {
    sign = -1.0;
  } else {
    sign  = 1.0;
  }

# include "generated/PriorEdges/QuaternionPrior_JErrNoises.cppready"

  static Eigen::Matrix<double, 3,3 > tmpCov;

  tmpCov = J*_noiseCov*J.transpose();
  ROAMmath::inv(tmpCov, _information);
}

void QuaternionPriorEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::QuaternionV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

# include "generated/PriorEdges/QuaternionPrior_Err.cppready"
}

void QuaternionPriorEdge::linearizeOplus() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::QuaternionV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

#include "generated/PriorEdges/QuaternionPrior_JErr.cppready"
}

std::string QuaternionPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "QuaternionPriorEdge(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
