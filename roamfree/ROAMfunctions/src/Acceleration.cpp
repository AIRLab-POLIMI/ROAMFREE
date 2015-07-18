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
 * Acceleration.cpp
 *
 *  Created on: Mar 28, 2013
 *      Author: davide
 */

#include "Acceleration.h"

namespace ROAMfunctions {

const bool AccelerationM::_usedComponents[] = { false, true, false, false, true,
		false, false, false, false, false };

const std::string AccelerationM::_paramsNames[] = { "G", "B" };

}
