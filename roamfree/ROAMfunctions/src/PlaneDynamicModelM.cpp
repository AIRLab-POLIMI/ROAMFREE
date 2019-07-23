/*
Copyright (c) 2013-2014 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (cucci@elet.polimi.it)
*/

/*
 * PlaneDynamicModelM.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: davide
 */

#include "PlaneDynamicModelM.h"

namespace ROAMfunctions {
const bool PlaneDynamicModelM::_usedComponents[] = {true, true, true, true, true, true, false, false, false, false};

const std::string PlaneDynamicModelM::_paramsNames[] = {"AirDensity", "FThurst", "FDrag", "FLat", "FLift", "MRoll", "MPitch", "MYaw", "CBar", "Cp", "Ibd", "Ibod", "Wind", "Gravity"};

} /* namespace ROAMfunctions */
