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
 * AnchoredRectangularObjectM.cpp
 *
 *  Created on: Jan 08, 2015
 *      Author: davide
 */

#include "AnchoredRectangularObjectM.h"

namespace ROAMfunctions {

const bool AnchoredRectangularObjectM::_usedComponents[] = {true, true, false, false, false, false, false, false, false, false};

const std::string AnchoredRectangularObjectM::_paramsNames[] = {"Dim", "F", "FOhp", "FOq", "CM"};

} /* namespace ROAMfunctions */
