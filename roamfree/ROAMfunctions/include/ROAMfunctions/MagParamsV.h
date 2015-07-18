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
 * MagParamsV.h
 *
 *  Created on: Jan 15, 2013
 *      Author: davide
 */

#ifndef MAGPARAMSV_H_
#define MAGPARAMSV_H_

#include "Variable.h"

namespace ROAMfunctions {

class MagParamsV {};

class MagParamsVI: public ROAMfunctions::Variable {
public:
  virtual ~MagParamsVI();

  virtual void setToOrigin(Eigen::VectorXd &x);
};

template<>
struct VariableTraits<MagParamsV> {

  static const unsigned int _INTERNAL_SIZE = 12;
  static const unsigned int _INCREMENT_SIZE = 12;

  typedef MagParamsVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* MAGPARAMSV_H_ */
