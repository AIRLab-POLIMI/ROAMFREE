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
 * Eucl3DRandomWalkProcessEdge.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: davide
 */

#include "PriorEdges/Eucl3DRandomWalkProcessEdge.h"

namespace ROAMestimation {

Eucl3DRandomWalkProcessEdge::Eucl3DRandomWalkProcessEdge() {
  _measurement.resize(3);
  _measurement.setZero(); // by default the prior on the derivative is zero
}

void Eucl3DRandomWalkProcessEdge::computeError() {

  const Eigen::VectorXd & older = static_cast<GenericVertex<
      ROAMfunctions::Eucl3DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & newer = static_cast<GenericVertex<
        ROAMfunctions::Eucl3DV> *>(_vertices[1])->estimate();

  _error = newer - older;
}

void Eucl3DRandomWalkProcessEdge::linearizeOplus() {
  // do nothing, _jacobianOplusXj and _jacobianOplusXi are constant
}

std::string Eucl3DRandomWalkProcessEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);
  g2o::OptimizableGraph::Vertex *x1 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[1]);

  s << "Eucl3DRandomWalkProcessEdge(" << x0->id() << "," << x1->id() << ")";

  return s.str();
}

}
