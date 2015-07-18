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
 * ObservationDescriptor.h
 *
 *  Created on: Dec 1, 2014
 *      Author: davide
 */

#ifndef OBSERVATIONDESCRIPTOR_H_
#define OBSERVATIONDESCRIPTOR_H_

#include <map>
#include <Eigen/Dense>

#include "ROAMestimation/ROAMestimation.h"

namespace ROAMvision {

class ObservationDescriptor {

public:
	double t;
	ROAMestimation::PoseVertexWrapper_Ptr pose;
	Eigen::Vector2d z;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

typedef std::map<double, ObservationDescriptor> ObservationMap;

} /* namespace ROAMvision */

#endif /* OBSERVATIONDESCRIPTOR_H_ */
