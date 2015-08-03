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
 * Matrix3DPriorEdge.cpp
 *
 *  Created on: Jul 11, 2013
 *      Author: davide
 */

#include "PriorEdges/Matrix3DPriorEdge.h"

namespace ROAMestimation {

Matrix3DPriorEdge::Matrix3DPriorEdge() {
  _measurement.resize(9);

  _jacobianOplusXi = Eigen::Matrix<double, 9, 9>::Identity();
}

void Matrix3DPriorEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Matrix3DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x - z;
}

void Matrix3DPriorEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string Matrix3DPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "Matrix3DPriorEdge(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
