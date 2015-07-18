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
 * QuaternionV.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#include "QuaternionV.h"

#include "ROAMmath/MathUtils.h"

#ifdef DEBUG_BUILD
#include <iostream>
#endif

namespace ROAMfunctions {

QuaternionVI::~QuaternionVI() {
}

void QuaternionVI::oplus(Eigen::VectorXd& newx, const Eigen::VectorXd& dx_in) {
  Eigen::VectorXd x = newx;

  Eigen::VectorXd dx = dx_in;

  // if norm([0, dx]) > 1 we have to normalize the increment
  double norm = std::pow(dx(0), 2) + std::pow(dx(1), 2) + std::pow(dx(2), 2);

  if (norm > 1.0) {
    dx(0) /= norm;
    dx(1) /= norm;
    dx(2) /= norm;
  }

  const static int _OFF = -1;

#ifdef DEBUG_BUILD
  // check for Nan and Inf BEFORE Oplus
  for (unsigned int i=0; i<newx.rows(); i++) {
    if (! ROAMmath::isProperDouble( newx(i) ) ) {
      std::cout << "NaN or Inf values in x!" << std::endl;
      std::cout << newx.transpose() << std::endl;
      assert(0);
    }
  }
#endif

#include "generated/Quaternion_OPLUS.cppready"

#ifdef DEBUG_BUILD
  // check for Nan and Inf AFTER Oplus
  for (unsigned int i=0; i<newx.rows(); i++) {
    if (! ROAMmath::isProperDouble( newx(i) ) ) {
      std::cout << "oplus(dx) caused NaN or Inf values in newx! " << std::endl;
      std::cout << dx.transpose() << std::endl;
      std::cout << newx.transpose() << std::endl;
      assert(0);
    }
  }
#endif
}

void QuaternionVI::setToOrigin(Eigen::VectorXd& x) {
  x(0) = 1;
  x(1) = 0;
  x(2) = 0;
  x(3) = 0;
}

} /* namespace ROAMfunctions */
