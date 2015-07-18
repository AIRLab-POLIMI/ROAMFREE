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
 * FactorGraphFilterFactory.h
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#ifndef FACTORGRAPHFILTERFACTORY_H_
#define FACTORGRAPHFILTERFACTORY_H_

namespace ROAMestimation {

class FactorGraphFilter;

class FactorGraphFilterFactory {

public:
	static FactorGraphFilter *getNewFactorGraphFilter();

};

} /* namespace ROAMestimation */

#endif /* FACTORGRAPHFILTERFACTORY_H_ */
