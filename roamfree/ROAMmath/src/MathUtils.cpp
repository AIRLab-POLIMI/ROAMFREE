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

 * MathUtils.cpp
 *
 *  Created on: Jan 9, 2013
 *      Author: davide
 */

#include "MathUtils.h"
#include <cmath>

namespace ROAMmath {

bool isProperDouble(const double l) {
  return !(l != l || std::abs(l) == std::numeric_limits<double>::infinity());
}

}
