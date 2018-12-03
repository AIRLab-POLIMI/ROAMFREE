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

#include "AccelerationInEarthFrame.h"

namespace ROAMfunctions {

const bool AccelerationInEarthFrameM::_usedComponents[] = { true, true, true, false, true,
		false, false, false, false, false };

const std::string AccelerationInEarthFrameM::_paramsNames[] = { "G", "B", "Gravity", "EarthRate", "EP" };

}
