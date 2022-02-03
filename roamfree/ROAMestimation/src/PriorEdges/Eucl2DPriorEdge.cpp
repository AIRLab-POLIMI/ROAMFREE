
/*
 * Eucl2DPriorEdge.cpp
 *
 *  Created on: Nov 28, 2018
 *      Author: Kenneth J Paul
 */

#include "PriorEdges/Eucl2DPriorEdge.h"

namespace ROAMestimation { 

Eucl2DPriorEdge::Eucl2DPriorEdge() {
  _measurement.resize(2);

  _jacobianOplusXi = Eigen::Matrix2d::Identity();
}

void Eucl2DPriorEdge::computeError() { 
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Eucl2DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x - z;
}

void Eucl2DPriorEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string Eucl2DPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "Euclidean2DPrior(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
