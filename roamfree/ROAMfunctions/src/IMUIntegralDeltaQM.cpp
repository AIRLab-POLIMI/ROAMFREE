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
 * IMUImtegralM.cpp
 *
 *  Created on: Sept 9, 2014
 *      Author: davide
 */

#include "IMUIntegralDeltaQM.h"

namespace ROAMfunctions {
const bool IMUImtegralDeltaQM::_usedComponents[] = {false, false, false, false, false, false, false, true, false, false};

const std::string IMUImtegralDeltaQM::_paramsNames[] = {"Bw"};

} /* namespace ROAMfunctions */
