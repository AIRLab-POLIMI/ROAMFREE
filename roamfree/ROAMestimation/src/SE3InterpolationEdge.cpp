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
 * SE3InterpolationEdge.cpp
 *
 *  Created on: Aug 31, 2016
 *      Author: davide
 */

#include "types.h"

#include "SE3InterpolationEdge.h"

#include "ROAMutils/StringUtils.h"

using namespace std;

namespace ROAMestimation {

SE3InterpolationEdge::SE3InterpolationEdge() {
  resize(4); // this edge is incident in three vertices

  _jacobianOplus[0].resize(6,6);
  _jacobianOplus[1].resize(6,6);
  _jacobianOplus[2].resize(6,6);
  _jacobianOplus[3].resize(6,1);
}

void SE3InterpolationEdge::init() {
  PoseVertex *x1v = static_cast<PoseVertex *>(_vertices[0]);
  PoseVertex *xiv = static_cast<PoseVertex *>(_vertices[1]);
  PoseVertex *x2v = static_cast<PoseVertex *>(_vertices[2]);
  
  GenericVertex<ROAMfunctions::Eucl1DV> *delayv = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[3]); 
  
  const Eigen::VectorXd & x1 = x1v->estimate();
  const Eigen::VectorXd & x2 = x2v->estimate();
  Eigen::VectorXd x(7);
  
  double delay = delayv->estimate()(0);
  
  double t1 = x1v->getTimestamp();
  double ti = xiv->getTimestamp();
  double t2 = x2v->getTimestamp();

# include "generated/SE3InterpolationEdge_Xhat.cppready"

  xiv->setEstimate(x);
}

void SE3InterpolationEdge::computeError() {
  PoseVertex *x1v = static_cast<PoseVertex *>(_vertices[0]);
  PoseVertex *xiv = static_cast<PoseVertex *>(_vertices[1]);
  PoseVertex *x2v = static_cast<PoseVertex *>(_vertices[2]);
  
  GenericVertex<ROAMfunctions::Eucl1DV> *delayv = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[3]); 

  const Eigen::VectorXd & x1 = x1v->estimate();
  const Eigen::VectorXd & xi = xiv->estimate();
  const Eigen::VectorXd & x2 = x2v->estimate();
  
  double delay = delayv->estimate()(0);

  double t1 = x1v->getTimestamp();
  double ti = xiv->getTimestamp();
  double t2 = x2v->getTimestamp();

# include "generated/SE3InterpolationEdge_Err.cppready"
}

void SE3InterpolationEdge::linearizeOplus() {
  PoseVertex *x1v = static_cast<PoseVertex *>(_vertices[0]);
  PoseVertex *xiv = static_cast<PoseVertex *>(_vertices[1]);
  PoseVertex *x2v = static_cast<PoseVertex *>(_vertices[2]);
  
  GenericVertex<ROAMfunctions::Eucl1DV> *delayv = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[3]); 

  const Eigen::VectorXd & x1 = x1v->estimate();
  const Eigen::VectorXd & xi = xiv->estimate();
  const Eigen::VectorXd & x2 = x2v->estimate();
  
  double delay = delayv->estimate()(0);

  double t1 = x1v->getTimestamp();
  double ti = xiv->getTimestamp();
  double t2 = x2v->getTimestamp();

  {
    Eigen::MatrixXd &J = _jacobianOplus[0];

#   include "generated/SE3InterpolationEdge_JErrX1.cppready"
  }

  {
    Eigen::MatrixXd &J = _jacobianOplus[1];

#   include "generated/SE3InterpolationEdge_JErrXI.cppready"
  }

  {
    Eigen::MatrixXd &J = _jacobianOplus[2];

#   include "generated/SE3InterpolationEdge_JErrX2.cppready"
  }
  
  {
    Eigen::MatrixXd &J = _jacobianOplus[3];

#   include "generated/SE3InterpolationEdge_JErrdelay.cppready"
  }
}

std::string SE3InterpolationEdge::writeDebugInfo() const {
  std::stringstream s;

  PoseVertex *x1v = static_cast<PoseVertex *>(_vertices[0]);
  PoseVertex *xiv = static_cast<PoseVertex *>(_vertices[1]);
  PoseVertex *x2v = static_cast<PoseVertex *>(_vertices[2]);

  s << "SE3InterpolationEdge[" << ROAMutils::StringUtils::writeNiceTimestamp(xiv->getTimestamp()) <<"](" << x1v->id() << "," << xiv->id() << ","
      << x2v->id() << ")";

  return s.str();
}

bool SE3InterpolationEdge::read(std::istream &s) {
  std::cerr
      << "[SE3InterpolationEdge] ImplementMe: read from g2o log not implemented"
      << std::endl;

  assert(false);

  return false;
}

bool SE3InterpolationEdge::write(std::ostream &s) const {
  std::cerr
      << "[SE3InterpolationEdge] ImplementMe: write to g2o log not implemented"
      << std::endl;

  assert(false);

  return false;
}

}
