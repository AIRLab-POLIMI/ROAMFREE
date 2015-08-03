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
 * FHPPriorOnHomogeneousPointEdge.cpp
 *
 *  Created on: Dec 2, 2014
 *      Author: davide
 */

#include "PriorEdges/FHPPriorOnHomogeneousPointEdge.h"

namespace ROAMestimation {

FHPPriorOnHomogeneousPointEdge::FHPPriorOnHomogeneousPointEdge() {
  _measurement.resize(2);

  _jacobianOplusXi = Eigen::Matrix<double, 2, 3>::Identity();
}

void FHPPriorOnHomogeneousPointEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Eucl3DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x.head(2) - z;
}

void FHPPriorOnHomogeneousPointEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string FHPPriorOnHomogeneousPointEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "FHPPriorOnHomogeneousPointEdge(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
