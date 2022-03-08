/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Kenneth Joseph Paul (kenneth.josephpaul@epfl.ch)    
*/

/*
 * Eucl1DGaussMarkovProcessEdge.cpp
 *
 *  Created on: July 27, 2021
 *      Author: kenneth
 */

#include "PriorEdges/Eucl1DGaussMarkovProcessEdge.h"

namespace ROAMestimation {

Eucl1DGaussMarkovProcessEdge::Eucl1DGaussMarkovProcessEdge() {
  _measurement.resize(1);
  _measurement.setZero(); // by default the prior on the derivative is zero
}

void Eucl1DGaussMarkovProcessEdge::computeError() {

  const Eigen::VectorXd & older = static_cast<GenericVertex<
      ROAMfunctions::Eucl1DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & newer = static_cast<GenericVertex<
        ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate();

  for (int i = 0; i < 1; i++) {
    _error(i) = newer(i) - exp(-_beta(i)*_dt) * older(i);
  }
}

void Eucl1DGaussMarkovProcessEdge::linearizeOplus() {
  // do nothing, _jacobianOplusXj and _jacobianOplusXi are constant
}

std::string Eucl1DGaussMarkovProcessEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);
  g2o::OptimizableGraph::Vertex *x1 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[1]);

  s << "Eucl1DGaussMarkovProcessEdge(" << x0->id() << "," << x1->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
