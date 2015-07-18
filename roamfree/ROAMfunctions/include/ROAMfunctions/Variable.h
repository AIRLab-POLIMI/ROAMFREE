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
 * Variable.h
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#ifndef VARIABLE_H_
#define VARIABLE_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class Variable {
public:
  virtual ~Variable();

  virtual void oplus(Eigen::VectorXd &newx, const Eigen::VectorXd &dx);
  virtual void setToOrigin(Eigen::VectorXd &x);

};

template<typename VariableType>
struct VariableTraits {
  // size of the internal representation
  static const unsigned int _INTERNAL_SIZE = 0;
  // size of the increment vector
  static const unsigned int _INCREMENT_SIZE = 0;
};

} /* namespace ROAMfunctions */
#endif /* VARIABLE_H_ */
