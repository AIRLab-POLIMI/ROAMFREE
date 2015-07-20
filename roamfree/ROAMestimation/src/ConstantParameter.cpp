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
 * ConstantParameter.cpp
 *
 *  Created on: May 16, 2013
 *      Author: davide
 */

#include "ConstantParameter.h"

namespace ROAMestimation {

ConstantParameter::ConstantParameter(g2o::AutoIDSparseOptimizer * opt,
    ParameterTypes typ, const std::string& name, double t0,
    const Eigen::VectorXd& x0) :
    ParameterVerticesManager(opt, typ, name) {

  newVertex(t0, x0);
}

ConstantParameter::ConstantParameter(g2o::AutoIDSparseOptimizer* opt,
    ParameterTypes typ, const std::string& name, GenericVertexInterface* v) :
    ParameterVerticesManager(opt, typ, name) {

  g2o::OptimizableGraph::Vertex *ov = v->getg2oOptGraphPointer();

  _isFixed = ov->fixed();
  _v[v->getTimestamp()] = ov;
}

bool ConstantParameter::getValueAt(double tstamp, Eigen::VectorXd &ret) const {
  // we have only one vertex and the value of the parameter is not dependent on the time
  GenericVertexInterface *v =
      dynamic_cast<GenericVertexInterface *>(_v.begin()->second);
  ret = v->getEstimate();

  // TODO: optimization, I could return !_v.begin()->second->fixed() but then if (e.g. from the viewer)
  // the _fixed property of the vertex is changed, someone has to notify to the edges that they may
  // have to re-evaluate this method.
  return true;
}

bool ConstantParameter::getJacobianAt(double tstamp, int j,
    Eigen::MatrixXd &ret) const {
  ret(0, 0) = 1;

  return false;
}

} /* namespace ROAMestimation */
