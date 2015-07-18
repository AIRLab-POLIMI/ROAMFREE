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
 * Variable.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#include "Variable.h"

namespace ROAMfunctions {

Variable::~Variable() {
}

void Variable::oplus(Eigen::VectorXd& newx, const Eigen::VectorXd& dx) {
  newx += dx;
}

void Variable::setToOrigin(Eigen::VectorXd& x) {
  x = Eigen::VectorXd::Zero(x.rows());
}

} /* namespace ROAMfunctions */
