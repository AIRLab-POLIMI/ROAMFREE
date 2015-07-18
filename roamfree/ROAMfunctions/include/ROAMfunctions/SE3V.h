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
 * SE3V.h
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#ifndef SE3V_H_
#define SE3V_H_

#include "Variable.h"

namespace ROAMfunctions {

class SE3V { };

class SE3VI {
public:
  virtual ~SE3VI();

  virtual void oplus(Eigen::VectorXd &newx, const Eigen::VectorXd &dx);
  virtual void setToOrigin(Eigen::VectorXd &x);

};

template<>
struct VariableTraits<SE3V> {

  static const unsigned int _INTERNAL_SIZE = 7;
  static const unsigned int _INCREMENT_SIZE = 6;

  typedef SE3VI VariableI;
};


} /* namespace ROAMfunctions */
#endif /* SE3V_H_ */
