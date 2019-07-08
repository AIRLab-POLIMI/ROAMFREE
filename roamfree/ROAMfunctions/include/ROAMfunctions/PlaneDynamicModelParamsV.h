/*
Copyright (c) 2013-2014 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (cucci@elet.polimi.it)
*/

/*
 * PlaneDynamicModelParamsV.h
 *
 *  Created on: Feb 10, 2015
 *      Author: davide
 */

#ifndef PLANEDYNAMICMODELPARAMSV_H_
#define PLANEDYNAMICMODELPARAMSV_H_

#include "Variable.h"

namespace ROAMfunctions {

class PlaneDynamicModelParamsV {
};

class PlaneDynamicModelParamsVI: public Variable {
public:
  virtual ~PlaneDynamicModelParamsVI();
};

template<>
struct VariableTraits<PlaneDynamicModelParamsV> {

  static const unsigned int _INTERNAL_SIZE = 30;
  static const unsigned int _INCREMENT_SIZE = 30;

  typedef PlaneDynamicModelParamsVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* PLANEDYNAMICMODELPARAMSV_H_ */
