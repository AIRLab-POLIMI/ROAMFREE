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
 * DisplacementM.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: davide
 */

#include "DisplacementM.h"

namespace ROAMfunctions {
const bool DisplacementM::_usedComponents[] = { false, false, false, false,
		false, false, true, true, false, false };

const std::string DisplacementM::_paramsNames[] = { "empty" };

} /* namespace ROAMfunctions */
