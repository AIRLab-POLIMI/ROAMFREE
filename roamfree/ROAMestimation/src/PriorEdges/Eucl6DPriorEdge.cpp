
/*
 * Eucl6DPriorEdge.cpp
 *
 *  Created on: Apr 20, 2026
 *      Author: Simon Gilgien
 */

#include "PriorEdges/Eucl6DPriorEdge.h"

namespace ROAMestimation {

Eucl6DPriorEdge::Eucl6DPriorEdge() {
  _measurement.resize(6);

  _jacobianOplusXi = Eigen::Matrix<double, 6, 6>::Identity();
}

void Eucl6DPriorEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Eucl6DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x - z;
}

void Eucl6DPriorEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string Eucl6DPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "Euclidean6DPrior(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
