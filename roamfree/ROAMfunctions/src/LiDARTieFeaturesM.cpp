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
 * LiDARTieFeaturesM.cpp
 *
 *  Created on: May 28, 2021
 *      Author: davide
 */

#include "LiDARTieFeaturesM.h"

namespace ROAMfunctions {                           // x      q      v      w      a  alpha  disp                qprev
const bool LiDARTieFeaturesM::_usedComponents[] = { false, true, false, false,false, false, true, false, false, true, false, false };

const std::string LiDARTieFeaturesM::_paramsNames[] = { };

}
