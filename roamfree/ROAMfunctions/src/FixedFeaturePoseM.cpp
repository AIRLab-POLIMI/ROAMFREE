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
 * FixedFeaturePoseM.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: davide
 */

#include "FixedFeaturePoseM.h"

namespace ROAMfunctions {
const bool FixedFeaturePoseM::_usedComponents[] = { true, true, false, false,
		false, false, false, false, false, false };

const std::string FixedFeaturePoseM::_paramsNames[] = { "Fposition",
		"Forientation" };

}
