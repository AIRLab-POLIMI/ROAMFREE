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
 * AnchoredRectangularObjectFirstM.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: davide
 */

#include "../include/ROAMfunctions/AnchoredRectangularObjectFirstM.h"

namespace ROAMfunctions {

const bool AnchoredRectangularObjectFirstM::_usedComponents[] = {true, true, false, false, false, false, false, false, false, false};

const std::string AnchoredRectangularObjectFirstM::_paramsNames[] = {"Dim", "FOhp", "FOq", "CM"};

} /* namespace ROAMfunctions */