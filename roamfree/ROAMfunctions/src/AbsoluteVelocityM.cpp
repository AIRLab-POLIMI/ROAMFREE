/*
Copyright (c) 2019 -2025 RPFL
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Kenneth Joseph Paul (kenneth.josephpaul@epfl.ch)
*/

/*
 * AbsoluteVelocityM.cpp
 *
 *  Created on: Jul 2, 2024
 *      Author: kenneth
 */

#include "AbsoluteVelocityM.h"

namespace ROAMfunctions {
    const bool AbsoluteVelocityM::_usedComponents[] = {
        false, // x
        true, // q
        true, // v
        false, // w
        false, // a
        false, // alpha
        false,
        false,
        false,
        false,
        false,
        false
    };

    const std::string AbsoluteVelocityM::_paramsNames[] = {

    };

}