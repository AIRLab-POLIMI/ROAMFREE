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
 * FHPFeatureHandler.h
 *
 *  Created on: Feb 16, 2015
 *      Author: dave
 */

#include "FHPFeatureHandler.h"

#include <map>

namespace ROAMvision {

class FHPFeatureHandlerBootstrap: public FHPFeatureHandler {
public:
	FHPFeatureHandlerBootstrap(double initialDepth);
	virtual bool addFeatureObservation(long int id, double t,
			const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

	void fixImmutableFeaturePoses(const Eigen::VectorXd &pose,
			double percentageThreshold, double minZDistance);

	inline bool bootstrapCompleted() const {
		return !_bootstrap;
	}

protected:
	bool initFeature(const std::string& sensor, const Eigen::VectorXd& z,
			ROAMestimation::PoseVertexWrapper_Ptr pv, long int id);

	void voteFixedPoseCandidates(
			std::map<double, unsigned int>& candidates,
			ObservationMap& map, double minZDistance);

	bool _bootstrap;
};

}
