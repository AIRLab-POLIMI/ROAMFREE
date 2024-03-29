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
 * IMUImtegralM.cpp
 *
 *  Created on: March 7, 2017
 *      Author: davide
 */

#include "QuadDynamicModelM.h"

namespace ROAMfunctions {                        // x      q      v      w      a    alpha
const bool QuadDynamicModelM::_usedComponents[] = {false, true,  true,  true, true, true, false, false, false, true, true, true};

const std::string QuadDynamicModelM::_paramsNames[] = {"WindXY", "WindZ", "DragModel", "LDrag", "QDrag", "RDrag", "Lambda", "Mu", "Mc", "Cp", "Ibd", "Ibod"};

} /* namespace ROAMfunctions */
