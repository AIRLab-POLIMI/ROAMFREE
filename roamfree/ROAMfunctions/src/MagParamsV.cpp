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
 * MagParamsV.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: davide
 */

#include "MagParamsV.h"

namespace ROAMfunctions {

MagParamsVI::~MagParamsVI() {}

void MagParamsVI::setToOrigin(Eigen::VectorXd& x) {
  x(0) = 1;
  x(1) = 0;
  x(2) = 0;
  x(3) = 0;
  x(4) = 1;
  x(5) = 0;
  x(6) = 0;
  x(7) = 0;
  x(8) = 1;
  x(9) = 0;
  x(10) = 0;
  x(11) = 0;
}

} /* namespace ROAMfunctions */
