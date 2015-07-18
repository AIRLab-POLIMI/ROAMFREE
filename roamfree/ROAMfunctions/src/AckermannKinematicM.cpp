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
 * AckermannM.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: davide
 */

#include "AckermannKinematicM.h"

namespace ROAMfunctions {

const bool AckermannM::_usedComponents[] = { false, false, true, true, false,
		false, false, false, false, false };

const std::string AckermannM::_paramsNames[] = { "kSpeed", "kSteer", "psiSteer",
		"L" };

} /* namespace ROAMestimation */
