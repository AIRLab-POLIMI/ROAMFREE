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
 * RectangularObjectM.cpp
 *
 *  Created on: Dec 02, 2014
 *      Author: davide
 */

#include "../include/ROAMfunctions/RectangularObjectM.h"

namespace ROAMfunctions {
const bool RectangularObjectM::_usedComponents[] = {true, true, false, false, false, false, false, false, false, false};

const std::string RectangularObjectM::_paramsNames[] = {"Dim", "F", "CM"};

} /* namespace ROAMfunctions */
