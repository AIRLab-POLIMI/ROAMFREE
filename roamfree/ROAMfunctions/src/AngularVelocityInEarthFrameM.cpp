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
 * AngularVelocity.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: davide
 */

#include "../include/ROAMfunctions/AngularVelocityInEarthFrameM.h"

namespace ROAMfunctions {
const bool AngularVelocityInEarthFrameM::_usedComponents[] = { false, true, false, true,
		false, false, false, false, false, false };

const std::string AngularVelocityInEarthFrameM::_paramsNames[] = { "G", "B", "EarthRate" };

}
