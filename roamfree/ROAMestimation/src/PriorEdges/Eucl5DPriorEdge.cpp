
/*
 * Eucl5DPriorEdge.cpp
 *
 *  Created on: Apr 22, 2026
 *      Author: Simon Gilgien
 */

#include "PriorEdges/Eucl5DPriorEdge.h"

namespace ROAMestimation {

Eucl5DPriorEdge::Eucl5DPriorEdge() {
  _measurement.resize(5);

  _jacobianOplusXi = Eigen::Matrix<double, 5, 5>::Identity();
}

void Eucl5DPriorEdge::computeError() {
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Eucl5DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x - z;
}

void Eucl5DPriorEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string Eucl5DPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "Euclidean5DPrior(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
