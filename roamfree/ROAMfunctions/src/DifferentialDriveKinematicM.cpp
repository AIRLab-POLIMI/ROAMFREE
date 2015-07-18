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
 * DifferentialDriveKinematicM.cpp
 *
 *  Created on: Jul 18, 2013
 *      Author: davide
 */

#include "DifferentialDriveKinematicM.h"

namespace ROAMfunctions {
const bool DifferentialDriveKinematicM::_usedComponents[] = { false, false,
		true, true, false, false, false, false, false, false };

const std::string DifferentialDriveKinematicM::_paramsNames[] = { "R", "L" };

} /* namespace ROAMfunctions */
