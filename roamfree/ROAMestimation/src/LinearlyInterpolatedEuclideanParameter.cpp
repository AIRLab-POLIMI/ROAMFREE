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
 * LinearlyInterpolatedEuclideanParameter.cpp
 *
 *  Created on: Apr 27, 2015
 *      Author: davide
 */

#include "LinearlyInterpolatedEuclideanParameter.h"

#include "ROAMutils/StringUtils.h"

namespace ROAMestimation {

LinearlyInterpolatedEuclideanParameter::LinearlyInterpolatedEuclideanParameter(
    double spacing, g2o::AutoIDSparseOptimizer* opt, ParameterTypes typ,
    const std::string& name, const Eigen::VectorXd& x0) :
    ParameterVerticesManager(opt, typ, name), _spacing(spacing), _x0(x0) {
}

std::map<double, g2o::OptimizableGraph::Vertex*>::const_iterator LinearlyInterpolatedEuclideanParameter::getVertices(
    double tstamp) const {
  return --_v.upper_bound(tstamp);
}

bool LinearlyInterpolatedEuclideanParameter::updateVertexSet(double mintstamp,
    double maxtstamp) {

  // we have no vertices yet, put one in the middle of the window
  if (_v.size() == 0) {
    newVertex(0.5 * (mintstamp + maxtstamp), _x0);
  }

  // new vertices forward
  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator last;
  while ((last = --_v.end())->first <= maxtstamp) {
    double newt = last->first + _spacing;

    // usual trick trough maps because at OptimizableGraph we cant directly
    // access to the estimate variable in the g2o vertex

    Eigen::Map<Eigen::VectorXd> last_estimate(
        last->second->accessEstimateData(), parameterEstimateDimension());

    g2o::OptimizableGraph::Vertex *newv = newVertex(newt, last_estimate);

#   ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr
        << "[LinearlyInterpolatedEuclideanParameter] Info: adding vertex at t="
        << ROAMutils::StringUtils::writeNiceTimestamp(newt) << " for " << _name
        << " initial guess: " << last_estimate.transpose() << ", pose window: ["
        << std::fixed << ROAMutils::StringUtils::writeNiceTimestamp(mintstamp)
        << ", " << ROAMutils::StringUtils::writeNiceTimestamp(maxtstamp) << "]"
        << std::endl;
#   endif
  }

  // new vertices backward
  std::map<double, g2o::OptimizableGraph::Vertex *>::iterator oldestVertex;
  while ((oldestVertex = _v.begin())->first > mintstamp) {

    double newt = oldestVertex->first - _spacing;

    Eigen::Map<Eigen::VectorXd> first_estimate(
        oldestVertex->second->accessEstimateData(),
        parameterEstimateDimension());

    g2o::OptimizableGraph::Vertex * newv = newVertex(newt, first_estimate);

#   ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr << "[LinearlyInterpolatedEuclideanParameter] Info: adding vertex at t=" << ROAMutils::StringUtils::writeNiceTimestamp(newt)
    << " for " << _name << " initial guess: " << first_estimate.transpose() << std::endl;
#   endif
  }

  /* erase old vertices
   while ((oldestVertex = _v.begin())->first <= mintstamp - _spacing) {
   scheduleDelete(oldestVertex->first, oldestVertex->second);
   _v.erase(oldestVertex);

   #   ifdef DEBUG_PRINT_INFO_MESSAGES
   std::cerr
   << "[LinearlyInterpolatedEuclideanParameter] Info: scheduling delete for vertex at t="
   << ROAMutils::StringUtils::writeNiceTimestamp(oldestVertex->first) << " for " << _name << std::endl;
   #   endif
   }
   */

  return true;
}

void LinearlyInterpolatedEuclideanParameter::prepareForPoseRemoval(
    double mintstamp, double maxtstamp) {

  // find the vertex for which the ts is strictly greather than maxtstamp
  auto firstNeeded = _v.upper_bound(maxtstamp);

  // this should never happen
  assert(firstNeeded != _v.begin());

  // the previous may still be needed for poses between its ts and mintstamp
  firstNeeded--;

  if (firstNeeded != _v.begin()) {
    VertexMap::iterator oldestVertex;
    while ((oldestVertex = _v.begin()) != firstNeeded) {
      scheduleDelete(oldestVertex->first, oldestVertex->second);
      _v.erase(oldestVertex);

#     ifdef DEBUG_PRINT_INFO_MESSAGES
      std::cerr
      << "[LinearlyInterpolatedEuclideanParameter] Info: scheduling delete for vertex at t="
      << ROAMutils::StringUtils::writeNiceTimestamp(oldestVertex->first) << " for " << _name << std::endl;
#     endif
    }
  }
}

bool LinearlyInterpolatedEuclideanParameter::getValueAt(double tstamp,
    Eigen::VectorXd& ret) const {

  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator it =
      getVertices(tstamp);

  double t1 = it->first;
  Eigen::Map<Eigen::VectorXd> x1(it->second->accessEstimateData(),
      parameterEstimateDimension());
  it++;

  double t2 = it->first;
  Eigen::Map<Eigen::VectorXd> x2(it->second->accessEstimateData(),
      parameterEstimateDimension());

  ret = x1 * (1 - (tstamp - t1) / _spacing) + x2 * (tstamp - t1) / _spacing;

  return true;
}

bool LinearlyInterpolatedEuclideanParameter::getJacobianAt(double tstamp, int j,
    Eigen::MatrixXd& ret) const {

  assert(j == 0 || j == 1);

  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator it =
      getVertices(tstamp);

  double t1 = it->first;

  if (j == 0) {
    ret(0) = (1 - (tstamp - t1) / _spacing);
  } else {
    ret(0) = (tstamp - t1) / _spacing;
  }

  return false; // this does NOT depend on the value of the parameter vertices

}

} /* namespace ROAMestimation */
