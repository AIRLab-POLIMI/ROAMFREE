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
 * Eucl1DPriorEdge.cpp
 *
 *  Created on: Jun 12, 2013
 *      Author: davide
 */

#include "PriorEdges/Eucl1DPriorEdge.h"

namespace ROAMestimation {

Eucl1DPriorEdge::Eucl1DPriorEdge() {
  _measurement.resize(1);

  _jacobianOplusXi(0,0) = 1;
}

void Eucl1DPriorEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Eucl1DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x - z;
}

void Eucl1DPriorEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string Eucl1DPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "Euclidean1DPrior(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
