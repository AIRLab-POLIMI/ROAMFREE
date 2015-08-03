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
 * SE3Prior.cpp
 *
 *  Created on: Apr 8, 2013
 *      Author: davide
 */

#include "PriorEdges/SE3PriorEdge.h"
#include "ROAMmath/MatricesOperations.h"

namespace ROAMestimation {

SE3PriorEdge::SE3PriorEdge() {
  _measurement.resize(7);
}

void SE3PriorEdge::afterVertexUpdate() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  static Eigen::VectorXd w(1);
  static Eigen::Matrix<double, 6, 6> J;

#include "generated/PriorEdges/SE3Prior_TestW.cppready"

  if (w(0) < 0) {
    sign = -1.0;
  } else {
    sign  = 1.0;
  }

#include "generated/PriorEdges/SE3Prior_JErrNoises.cppready"

  static Eigen::Matrix<double, 6,6 > tmpCov;

  tmpCov = J*_noiseCov*J.transpose();
  ROAMmath::inv(tmpCov, _information);
}

void SE3PriorEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

#include "generated/PriorEdges/SE3Prior_Err.cppready"
}

void SE3PriorEdge::linearizeOplus() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

#include "generated/PriorEdges/SE3Prior_JErr.cppready"
}

std::string SE3PriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "SE3PriorEdge(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
