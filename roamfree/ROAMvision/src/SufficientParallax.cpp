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
 * SufficientParallax.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: davide
 */

#include "SufficientParallax.h"

#include <iostream>

using namespace std;

namespace ROAMvision {

SufficientParallax::SufficientParallax(double minParallax, double initialDepth,
		const ObservationMap& zHistory, const double *K) :
		FHPInitializationStrategy(zHistory, K), _minParallax(minParallax), _initialDepth(
				initialDepth), _traveled(0.0) {
}

bool SufficientParallax::initialize(Eigen::VectorXd& HP) {

  // TODO: we redo all the computation each time to be sure

  if (_zHistory.size() >= 2) {

    auto it = _zHistory.begin();
    const ObservationDescriptor *last = &(it->second);

    // put the viewing ray in world frame
    Eigen::Vector3d rCnorm, rWnorm;

    const Eigen::VectorXd &T_WC_0 = last->pose->getEstimate();

    Eigen::Vector3d z0;
    z0 << last->z(0), last->z(1), 1.0;
    rCnorm << _K.inverse() * z0;
    rCnorm.normalize();

    Eigen::Quaterniond q_WC0(T_WC_0(3),T_WC_0(4),T_WC_0(5),T_WC_0(6));
    rWnorm = q_WC0._transformVector(rCnorm);

    // project the distance on a plane orthogonal to the viewing ray and take the maximum

    while (++it != _zHistory.end() && it->second.pose->hasBeenEstimated() == true) {
      const ObservationDescriptor *cur = &(it->second);

      Eigen::Vector3d distW = last->pose->getEstimate().head(3) - cur->pose->getEstimate().head(3);

      // project the distance
      double distOrth = (distW - distW.dot(rWnorm)*rWnorm).norm();

      _traveled = max (_traveled, distOrth);
    }

	}

	if (_traveled > _minParallax && _zHistory.size() >= 3) {
		const ObservationDescriptor &first = _zHistory.begin()->second;

		Eigen::Vector3d z0;
		z0 << first.z(0), first.z(1), 1.0;

		HP << _K.inverse() * z0;
		HP(2) = 1.0 / _initialDepth; // 1/d distance of the plane parallel to the image plane on which features are initialized.

#		ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
		cerr << fixed << "[SufficientParallax] traveled " << _traveled << " m, HP initialization: " << HP.transpose() << endl;
#		endif

		return true;
	}

	return false;
}

}
