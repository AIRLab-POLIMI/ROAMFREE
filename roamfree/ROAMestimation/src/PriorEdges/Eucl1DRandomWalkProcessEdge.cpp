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
 * Eucl2DGaussMarkovProcessEdge.cpp
 *
 *  Created on: July 27, 2021
 *      Author: kenneth
 */


#include "PriorEdges/Eucl1DRandomWalkProcessEdge.h"

namespace ROAMestimation {

Eucl1DRandomWalkProcessEdge::Eucl1DRandomWalkProcessEdge() {
  _measurement.resize(1);
  _measurement.setZero(); // by default the prior on the derivative is zero
}

void Eucl1DRandomWalkProcessEdge::computeError() {

  const Eigen::VectorXd & older = static_cast<GenericVertex<
      ROAMfunctions::Eucl1DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & newer = static_cast<GenericVertex<
        ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate();

  _error = newer - older;
}

void Eucl1DRandomWalkProcessEdge::linearizeOplus() {
  // do nothing, _jacobianOplusXj and _jacobianOplusXi are constant
}

std::string Eucl1DRandomWalkProcessEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);
  g2o::OptimizableGraph::Vertex *x1 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[1]);

  s << "Eucl1DRandomWalkProcessEdge(" << x0->id() << "," << x1->id() << ")";

  return s.str();
}

}
