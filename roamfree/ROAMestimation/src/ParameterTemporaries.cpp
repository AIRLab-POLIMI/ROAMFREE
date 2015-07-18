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
 * ParameterTemporaries.cpp
 *
 *  Created on: May 20, 2013
 *      Author: davide
 */

#include "ParameterTemporaries.h"

namespace ROAMestimation {

ParameterTemporaries::ParameterTemporaries(
    boost::shared_ptr<ParameterVerticesManager> inp, double ts) :
    p(inp), value(inp->parameterEstimateDimension()), _hasToUpdateJacobian(true), _hasToUpdateValue(
        true), _tstamp(ts) {

  for (int k = 0; k < p->getWindowSize(); k++) {
    // I push getWindowSize() dynamic matrices. I expect these will be resized from
    // the parameter evaluator (they could shrink to scalars..)
    jacobians.emplace_back();
    p->resizeJacobianMatrix(jacobians.back());
  }

}

void ParameterTemporaries::updateTemporaries() {
  if (_hasToUpdateValue) {
    _hasToUpdateValue = p->getValueAt(_tstamp, value);
  }

  if (_hasToUpdateJacobian) {
    for (int k = 0; k < p->getWindowSize(); k++) {
      _hasToUpdateJacobian = p->getJacobianAt(_tstamp, k, jacobians[k]);
    }
  }
}

} /* namespace ROAMestimation */
