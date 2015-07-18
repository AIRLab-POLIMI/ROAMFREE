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
 * FHPTrackDescriptor.h
 *
 *  Created on: Nov 27, 2014
 *      Author: davide
 */

#ifndef FHPTRACKDESCRIPTOR_H_
#define FHPTRACKDESCRIPTOR_H_

#include <map>

#include <Eigen/Dense>

#include "ROAMestimation/ROAMestimation.h"
#include "ObservationDescriptor.h"
#include "FHPInitializationStrategy.h"

namespace ROAMvision {

class FHPTrackDescriptor {

public:

	bool isInitialized;
	FHPInitializationStrategy *initStrategy;

	ObservationMap zHistory;

	ROAMestimation::PoseVertexWrapper_Ptr anchorFrame, lastFrame;

};

} /* namespace ROAMvision */

#endif /* FHPTRACKDESCRIPTOR_H_ */
