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
 * SE3V.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#include "SE3V.h"

#include "ROAMmath/MathUtils.h"

#ifdef DEBUG_BUILD
#include <iostream>
#endif

namespace ROAMfunctions {

SE3VI::~SE3VI() {
}

void SE3VI::oplus(Eigen::VectorXd& newx, const Eigen::VectorXd& dx_in) {
  Eigen::VectorXd x = newx;

  Eigen::VectorXd dx = dx_in;

  // if norm([0, dx]) > 1 we have to normalize the increment
  double norm = std::pow(dx(3), 2) + std::pow(dx(4), 2) + std::pow(dx(5), 2);

  if (norm > 1.0) {
    dx(3) /= norm;
    dx(4) /= norm;
    dx(5) /= norm;
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

#include "generated/SE3_OPLUS.cppready"

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

void SE3VI::setToOrigin(Eigen::VectorXd& x) {
  x(0) = 0;
  x(1) = 0;
  x(2) = 0;
  x(3) = 1;
  x(4) = 0;
  x(5) = 0;
  x(6) = 0;
}

} /* namespace ROAMfunctions */
