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
 * ImageFeatureHandler.h
 *
 *  Created on: Nov 27, 2014
 *      Author: davide
 */

#ifndef IMAGEFEATUREHANDLER_H_
#define IMAGEFEATUREHANDLER_H_

#include "ROAMestimation/ROAMestimation.h"

namespace ROAMvision {

class ImageFeatureHandler {

public:

	virtual ~ImageFeatureHandler() {
	}
	;

	ImageFeatureHandler() :
			_timestampOffsetTreshold(std::numeric_limits<double>::infinity()) {
	}
	;

	virtual bool init(ROAMestimation::FactorGraphFilter* f,
			const std::string &name, const Eigen::VectorXd & T_OS,
			const Eigen::VectorXd & K) = 0;
	virtual bool addFeatureObservation(long int id, double t,
			const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) = 0;

	virtual bool getFeaturePositionInWorldFrame(long int id,
			Eigen::VectorXd &lw) const = 0;
	virtual bool getFeaturesIds(std::vector<long int> &to) const = 0;
	virtual long int getNActiveFeatures() const = 0;

	virtual void setTimestampOffsetTreshold(double dt) = 0;

	virtual void fixOlderPosesWRTVisibleFeatures() = 0;

protected:

	std::string _sensorName;
	double _timestampOffsetTreshold;
	ROAMestimation::FactorGraphFilter* _filter;

};

} /* namespace ROAMvision */

#endif /* IMAGEFEATUREHANDLER_H_ */
