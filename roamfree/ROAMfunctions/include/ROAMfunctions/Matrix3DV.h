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
 * Matrix3DV.h
 *
 *  Created on: Jan 21, 2013
 *      Author: davide
 */

#ifndef MATRIX3DV_H_
#define MATRIX3DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Matrix3DV {};

class Matrix3DVI: public ROAMfunctions::Variable {
public:
  virtual ~Matrix3DVI();

  virtual void setToOrigin(Eigen::VectorXd &x);
};

template<>
struct VariableTraits<Matrix3DV> {

  static const unsigned int _INTERNAL_SIZE = 9;
  static const unsigned int _INCREMENT_SIZE = 9;

  typedef Matrix3DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* MATRIX3DV_H_ */
