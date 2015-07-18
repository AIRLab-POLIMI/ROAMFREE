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
 * FHPInitializationStrategy.h
 *
 *  Created on: Dec 1, 2014
 *      Author: davide
 */

#ifndef FHPINITIALIZATIONSTRATEGY_H_
#define FHPINITIALIZATIONSTRATEGY_H_

#include <Eigen/Dense>

#include "ObservationDescriptor.h"

namespace ROAMvision {

class FHPInitializationStrategy {

public:

	FHPInitializationStrategy(const ObservationMap &zHistory, const double *K);

	virtual ~FHPInitializationStrategy();

	virtual bool initialize(Eigen::VectorXd &HP) = 0;

protected:
	const ObservationMap &_zHistory;
	const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> _K;

};

} /* namespace ROAMvision */

#endif /* FHPINITIALIZATIONSTRATEGY_H_ */
