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
 * ParameterVertexWrapperImpl.cpp
 *
 *  Created on: May 13, 2014
 *      Author: davide
 */

#include "ParameterWrapperImpl.h"

#include "g2o/core/optimizable_graph.h"
#include "GenericVertexInterface.h"

namespace ROAMestimation {

ParameterWrapper_Impl::ParameterWrapper_Impl(ParameterVerticesManager* param) :
		_param(param) {
}

const Eigen::VectorXd& ParameterWrapper_Impl::getEstimate() const {
	return getEstimate(0.0);
}

const Eigen::VectorXd& ParameterWrapper_Impl::getEstimate(
		double t) const {
	return _param->getVertexEstimate(t);
}

void ParameterWrapper_Impl::setEstimate(
    const Eigen::VectorXd& x, double t) {
  _param->setVertexEstimate(0.0, x);
}

void ParameterWrapper_Impl::setEstimate(
    const Eigen::VectorXd& x) {
  setEstimate(x, 0.0);
}

void ParameterWrapper_Impl::setEstimate(double x, double t) {
  Eigen::VectorXd tmp(1);
  tmp << x;

  setEstimate(tmp, t);
}

void ParameterWrapper_Impl::setEstimate(double x) {
  Eigen::VectorXd tmp(1);
  tmp << x;

  setEstimate(tmp, 0.0);
}

void ParameterWrapper_Impl::getValueAt(Eigen::VectorXd& ret, double t) {
  _param->getValueAt(t, ret);
}

} /* namespace ROAMestimation */
