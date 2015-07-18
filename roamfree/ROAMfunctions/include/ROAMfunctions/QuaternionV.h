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
 * QuaternionV.h
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#ifndef QUATERNIONV_H_
#define QUATERNIONV_H_

#include "Variable.h"

namespace ROAMfunctions {

class QuaternionV { };

class QuaternionVI : public Variable {
public:
  virtual ~QuaternionVI();

  virtual void oplus(Eigen::VectorXd &newx, const Eigen::VectorXd &dx);
  virtual void setToOrigin(Eigen::VectorXd &x);
};

template<>
struct VariableTraits<QuaternionV> {

  static const unsigned int _INTERNAL_SIZE = 4;
  static const unsigned int _INCREMENT_SIZE = 3;

  typedef QuaternionVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* QUATERNIONV_H_ */
