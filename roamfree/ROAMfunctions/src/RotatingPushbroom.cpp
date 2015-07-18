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
 * RotatingPushbroomM.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: davide
 */

#include "../include/ROAMfunctions/RotatingPushbroomM.h"

namespace ROAMfunctions {

const bool RotatingPushbroomM::_usedComponents[] = {true, true, false, false, false, false, true, true, false, false};

const std::string RotatingPushbroomM::_paramsNames[] = {"H", "K"};

} /* namespace ROAMfunctions */
