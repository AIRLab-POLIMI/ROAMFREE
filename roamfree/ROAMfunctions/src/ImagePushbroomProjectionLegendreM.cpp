/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)
    Simon Gilgien (simon.gilgien@epfl.ch
*/

/*
 * ImagePushBroomProjectionLegendreM.cpp
 *
 *  Created on: May 27, 2026
 *      Author: simon
 */

#include "ImagePushbroomProjectionLegendreM.h"

namespace ROAMfunctions {
const bool ImagePushbroomProjectionLegendreM::_usedComponents[] = {true, true, false, false, false, false, false, false, false, false, false, false};

const std::string ImagePushbroomProjectionLegendreM::_paramsNames[] = {"Lw", "CM", "AD", "ND", "SW"};

} /* namespace ROAMfunctions */
