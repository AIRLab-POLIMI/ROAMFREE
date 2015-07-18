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
 *  Created on: Nov 27, 2014
 *      Author: davide
 */

#ifndef FHPFEATUREHANDLER_H_
#define FHPFEATUREHANDLER_H_

#include <map>

#include "ImageFeatureHandler.h"

#include "FHPTrackDescriptor.h"

namespace ROAMvision {

class FHPFeatureHandler: public ImageFeatureHandler {

public:
	FHPFeatureHandler(double initialDepth);

	virtual bool init(ROAMestimation::FactorGraphFilter* f,
			const std::string &name, const Eigen::VectorXd & T_OS,
			const Eigen::VectorXd & K);
	virtual bool addFeatureObservation(long int id, double t,
			const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

	virtual bool getFeaturePositionInWorldFrame(long int id,
				Eigen::VectorXd &lw) const;
	virtual bool getFeaturesIds(std::vector<long int> &to) const;
	virtual long int getNActiveFeatures() const;

	virtual void setTimestampOffsetTreshold(double dt);

	virtual void fixOlderPosesWRTVisibleFeatures();

protected:

	virtual bool initFeature(const std::string& sensor, const Eigen::VectorXd& z,
			ROAMestimation::PoseVertexWrapper_Ptr pv, long int id);

	std::string getFeatureSensor(long int id) const;

	inline const ObservationDescriptor &getNewestObservation(long int id) const {
		assert(_features.count(id) == 1);

		return _features.find(id)->second.zHistory.rbegin()->second;
	}

	typedef std::map<long int, FHPTrackDescriptor> FeatureMap;

	double _initialDepth;

	FeatureMap _features;

};

} /* namespace ROAMvision */

#endif /* FHPFEATUREHANDLER_H_ */
