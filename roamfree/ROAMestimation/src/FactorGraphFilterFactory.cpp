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
 * FactorGraphFilterFactory.cpp
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#include "FactorGraphFilterFactory.h"

#include "FactorGraphFilterImpl.h"

namespace ROAMestimation {

FactorGraphFilter* FactorGraphFilterFactory::getNewFactorGraphFilter() {
	return new FactorGraphFilter_Impl;
}


} /* namespace ROAMestimation */
