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
 * EuclideanTrackDescriptor.h
 *
 *  Created on: Jan 6, 2016
 *      Author: davide
 */

#ifndef EUCLIDEANTRACKDESCRIPTOR_H_
#define EUCLIDEANTRACKDESCRIPTOR_H_

#include <Eigen/Dense>

#include "ROAMestimation/ROAMestimation.h"
#include "ObservationDescriptor.h"

namespace ROAMvision {

class EuclideanTrackDescriptor {

  public:

    EuclideanTrackDescriptor() :
        isInitialized(false) {
    }

    bool isInitialized;

    ObservationMap zHistory;
};

} /* namespace ROAMvision */

#endif /* EUCLIDEANTRACKDESCRIPTOR_H_ */
