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
 * FHPInitializationStrategy.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: davide
 */

#include "FHPInitializationStrategy.h"

namespace ROAMvision {

FHPInitializationStrategy::FHPInitializationStrategy(
		const ObservationMap& zHistory, const double *K) :
		_zHistory(zHistory), _K(K) {
}

FHPInitializationStrategy::~FHPInitializationStrategy() {
}

} /* namespace ROAMvision */
