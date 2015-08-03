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
 * SpatialIndex.cpp
 *
 *  Created on: Mar 26, 2014
 *      Author: davide
 */

#include "../../include/ROAMestimation/SpatialIndex/SpatialIndex.h"
#include "PoseVertexWrapperImpl.h"

namespace ROAMestimation {

void SpatialIndex::build(const PoseMap& poses) {

	// trash anything I might have
	_index.clear();

	// iterate through the PoseMap and copy the vertices for which we have to keep a spatial index
	for (PoseMap::const_iterator it = poses.begin(); it != poses.end(); ++it) {
		PoseVertexWrapper_Impl v(it->second);
		if (v.getKeepSpatialIndex() == true) {
			_index[it->first] = it->second;
		}
	}

# ifdef DEBUG_PRINT_INFO_MESSAGES
	std::cerr << "[SpatialIndex] Info: build finished, " << _index.size() << " poses indexed." << std::endl;
# endif
}

double SpatialIndex::euclidean2DDistance(const Eigen::VectorXd &x1,
		const Eigen::VectorXd &x2) const {
	return std::sqrt(std::pow(x1(0) - x2(0), 2) + std::pow(x1(1) - x2(1), 2));
}

} /* namespace ROAMestimation */
