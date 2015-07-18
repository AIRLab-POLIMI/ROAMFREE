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
 * StringUtils.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: davide
 */

#include "StringUtils.h"

#include <sstream>
#include <cmath>

namespace ROAMutils {

std::string StringUtils::writeNiceTimestamp(double t) {
	std::stringstream s;
	s << std::fixed << fmod(t, 100);
	return s.str();
}

} /* namespace ROAMutils */

