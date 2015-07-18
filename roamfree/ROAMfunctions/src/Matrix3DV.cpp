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
 * Matrix3DV.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: davide
 */

#include "Matrix3DV.h"

namespace ROAMfunctions {

Matrix3DVI::~Matrix3DVI() {
}

void Matrix3DVI::setToOrigin(Eigen::VectorXd& x) {
  x(0) = 1;
  x(1) = 0;
  x(2) = 0;
  x(3) = 0;
  x(4) = 1;
  x(5) = 0;
  x(6) = 0;
  x(7) = 0;
  x(8) = 1;
}

} /* namespace ROAMfunctions */

