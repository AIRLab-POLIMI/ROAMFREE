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
 * SufficientParallax.h
 *
 *  Created on: Feb 16, 2015
 *      Author: davide
 */

#ifndef SUFFICIENTZCHANGE_H_
#define SUFFICIENTZCHANGE_H_

#include "FHPInitializationStrategy.h"

namespace ROAMvision {

class SufficientParallax: public FHPInitializationStrategy {

public:
  SufficientParallax(double minParallax, double initialDepth,
			const ObservationMap &zHistory, const double *K);

	virtual bool initialize(Eigen::VectorXd &HP);

protected:
	double _traveled;
	double _initialDepth;
	double _minParallax;
};

} /* namespace ROAMvision */

#endif /* SUFFICIENTZCHANGE_H_ */
