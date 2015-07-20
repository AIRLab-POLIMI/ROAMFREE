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
 * LimitedBandwithParameter.cpp
 *
 *  Created on: May 16, 2013
 *      Author: davide
 */

#include "LimitedBandwithEuclideanParameter.h"

#include "ROAMutils/StringUtils.h"

namespace ROAMestimation {

LimitedBandwithEuclideanParameter::LimitedBandwithEuclideanParameter(
    double band, int a, g2o::AutoIDSparseOptimizer * opt, ParameterTypes typ,
    const std::string& name, const Eigen::VectorXd& x0) :
    ParameterVerticesManager(opt, typ, name), _bandwith(band), _a(a), _x0(x0) {
}

std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator LimitedBandwithEuclideanParameter::getVertices(
    double tstamp) const {
// it returns the first vertex for which the timestamp is GREATHER OR EQUAL to arg
  return _v.lower_bound(tstamp + (float) (-_a) / (2.0 * _bandwith));
}

bool LimitedBandwithEuclideanParameter::updateVertexSet(double mintstamp,
    double maxtstamp) {

  // we have no vertices yet, put one in the middle of the window
  if (_v.size() == 0) {
    newVertex(0.5 * (mintstamp + maxtstamp), _x0);
  }

// insert new vertices if needed
  double deltat = 1.0 / (2.0 * _bandwith);

// new vertices forward
  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator last;
  while ((last = --_v.end())->first <= maxtstamp + (float) (_a - 1) * deltat) {
    double newt = last->first + deltat;

    // usual trick trough maps because at OptimizableGraph we cant directly
    // access to the estimate variable in the g2o vertex

    Eigen::Map<Eigen::VectorXd> last_estimate(
        last->second->accessEstimateData(), parameterEstimateDimension());

    newVertex(newt, last_estimate);

#		ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr << "[LimitedBandwidthParameter] Info: adding vertex at t=" << ROAMutils::StringUtils::writeNiceTimestamp(newt)
    << " for " << _name << " initial guess: " << last_estimate.transpose() << std::endl;
#		endif
  }

// new vertices backward
  std::map<double, g2o::OptimizableGraph::Vertex *>::iterator first;
  while ((first = _v.begin())->first >= mintstamp + (float(-_a + 1) * deltat)) {

    double newt = first->first - deltat;

    Eigen::Map<Eigen::VectorXd> first_estimate(
        first->second->accessEstimateData(), parameterEstimateDimension());

    newVertex(newt, first_estimate);

#		ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr << "[LimitedBandwidthParameter] Info: adding vertex at t=" << ROAMutils::StringUtils::writeNiceTimestamp(newt)
    << " for " << _name << " initial guess: " << first_estimate.transpose() << std::endl;
#		endif
  }

  /* erase old vertices
  while ((first = _v.begin())->first < mintstamp + (float(-_a) * deltat)) {
    scheduleDelete(first->first, first->second);
    _v.erase(first);

#	  ifdef DEBUG_PRINT_INFO_MESSAGES
    std::cerr
    << "[LimitedBandwidthParameter] Info: scheduling delete for vertex at t="
    << ROAMutils::StringUtils::writeNiceTimestamp(first->first) << " for " << _name << std::endl;
#		endif
  }
  //*/
  return true;
}

void LimitedBandwithEuclideanParameter::prepareForPoseRemoval(
    double mintstamp, double maxtstamp) {
  std::cerr << "[LimitedBandwidthParameter] implementMe: prepareForPoseRemoval() " << std::endl;

  //TODO: implement this
  assert(false);
}

bool LimitedBandwithEuclideanParameter::getValueAt(double tstamp,
    Eigen::VectorXd& ret) const {

// here I convolve the 2*a samples in the vertex window with the Lanczos functions
// N.B. this thing is only meaningful if the parameter spans an Euclidean space
// and its components are independent. The interpolation is done component-by-component!

  ret.resize(parameterEstimateDimension());
  ret.setZero();

  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator it =
      getVertices(tstamp);

// for each parameter sample
  for (int k = 0; k < getWindowSize(); k++, it++) {
    g2o::OptimizableGraph::Vertex *v = it->second;
    Eigen::Map<Eigen::VectorXd> value(v->accessEstimateData(),
        parameterEstimateDimension());

    double t = tstamp - it->first;
    double x = t * 2.0 * _bandwith;

    // machinery to employ mathematica output
    Eigen::Matrix<double, 1, 1> f;
    const static int _OFF = -1;

    // for this sample compute the height of the Lanczos function a time t
    if (std::fabs(x) > 0.05) {
#			include "generated/Lanczos_f.cppready"
    } else {
      //here we compute the window with Taylor series up to the sixt order
#			include "generated/ApproxLanczos_f.cppready"
    }

    //std::cout << "t: " << t << " x: " << x << " f: " << f << std::endl;

    ret += f(0) * value;
  }

  return true; // this depends on the value of the parameter vertices
}

bool LimitedBandwithEuclideanParameter::getJacobianAt(double tstamp, int j,
    Eigen::MatrixXd &ret) const {
  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator it =
      getVertices(tstamp);

  for (int k = 0; k < j; k++, it++)
    ;

  double t = tstamp - it->first;
  double x = t * 2.0 * _bandwith;

// machinery to employ mathematica output
  Eigen::Matrix<double, 1, 1> f;
  const static int _OFF = -1;

// for this sample compute the height of the Lanczos function a time t
  if (std::fabs(x) > 0.05) {
#		include "generated/Lanczos_f.cppready"
  } else {
    //here we compute the window with Taylor series up to the sixt order
#		include "generated/ApproxLanczos_f.cppready"
  }

//std::cout << "t: " << t << " x: " << x << " f: " << f << std::endl;

  ret = f;

  return false; // this does NOT depend on the value of the parameter vertices
}

} /* namespace ROAMestimation */
