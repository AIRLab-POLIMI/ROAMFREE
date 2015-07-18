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
 * AbsolutePositionM.cpp
 *
 *  Created on: Sept 11, 2014
 *      Author: davide
 */

#include "AbsolutePoseM.h"

namespace ROAMfunctions {

const bool AbsolutePoseM::_usedComponents[] = { true, true, false, false,
		false, false, false, false, false, false };

const std::string AbsolutePoseM::_paramsNames[] = { "empty" };

}
