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
 * AckermannKinematicNoInputsM.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: davide
 */

#include "AckermannKinematicNoInputsM.h"

namespace ROAMfunctions {

const bool AckermannKinematicNoInputsM::_usedComponents[] = { false, false, true, true, false,
    false, false, false, false, false };

const std::string AckermannKinematicNoInputsM::_paramsNames[] = { "CV", "CDelta", "L" };

} /* namespace ROAMestimation */
