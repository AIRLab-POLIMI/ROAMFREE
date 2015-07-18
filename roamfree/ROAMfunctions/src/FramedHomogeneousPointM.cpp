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
 * FramedHomogeneousPointM.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: davide
 */

#include "FramedHomogeneousPointM.h"

namespace ROAMfunctions {
const bool FramedHomogeneousPointM::_usedComponents[] = {true, true, false, false, false, false, false, false, false, false};

const std::string FramedHomogeneousPointM::_paramsNames[] = {"HP", "F", "CM"};

} /* namespace ROAMfunctions */
