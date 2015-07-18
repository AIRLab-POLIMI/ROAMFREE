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
 * VectorFieldAsCompass.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: davide
 */

#include "VectorFieldAsCompass.h"

namespace ROAMfunctions {

const bool VectorFieldAsCompassM::_usedComponents[] = { false, true, false,
		false, false, false, false, false, false, false };

const std::string VectorFieldAsCompassM::_paramsNames[] = { "h" };

} /* namespace ROAMfunctions */
