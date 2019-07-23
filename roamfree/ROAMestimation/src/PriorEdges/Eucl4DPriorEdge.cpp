
/*
 * Eucl4DPriorEdge.cpp
 *
 *  Created on: Jul 17, 2019
 *      Author: Kenneth J Paul
 */

#include "PriorEdges/Eucl4DPriorEdge.h"

namespace ROAMestimation { 

Eucl4DPriorEdge::Eucl4DPriorEdge() {
  _measurement.resize(4);

  _jacobianOplusXi = Eigen::Matrix4d::Identity();
}

void Eucl4DPriorEdge::computeError() { 
  const Eigen::VectorXd & x = static_cast<GenericVertex<
      ROAMfunctions::Eucl4DV> *>(_vertices[0])->estimate();
  const Eigen::VectorXd & z = _measurement;

  _error = x - z;
}

void Eucl4DPriorEdge::linearizeOplus() {

  // do nothing, the jacobian is always the identity function
}

std::string Eucl4DPriorEdge::writeDebugInfo() const {
  std::stringstream s;

  g2o::OptimizableGraph::Vertex *x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

  s << "Euclidean4DPrior(" << x0->id() << ")";

  return s.str();
}

} /* namespace ROAMestimation */
