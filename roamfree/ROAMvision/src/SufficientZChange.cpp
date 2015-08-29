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
 * SufficientZChange.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: davide
 */

#include "SufficientZChange.h"

#include <iostream>

using namespace std;

namespace ROAMvision {

SufficientZChange::SufficientZChange(double minZChange, double initialDepth,
		const ObservationMap& zHistory, const double *K) :
		FHPInitializationStrategy(zHistory, K), _minZChange(minZChange), _initialDepth(
				initialDepth), _zChange(0.0) {
}

bool SufficientZChange::initialize(Eigen::VectorXd& HP) {

	if (_zHistory.size() >= 2) {

	  auto it = _zHistory.begin();
	  const ObservationDescriptor *first = &(it->second);

		while (++it != _zHistory.end()) {
		_zChange = max(_zChange, (it->second.z - first->z).norm());
		}
	}

	if (_zChange > _minZChange && _zHistory.size() >= 5) {
		const ObservationDescriptor &first = _zHistory.begin()->second;

		Eigen::Vector3d z0;
		z0 << first.z(0), first.z(1), 1.0;

		HP << _K.inverse() * z0;
		HP(2) = 1.0 / _initialDepth; // 1/d distance of the plane parallel to the image plane on which features are initialized.

#		ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
		cerr << fixed << "[SufficientZChange] HP initialization: " << HP.transpose() << endl;
#		endif

		return true;
	}

	return false;
}

}
