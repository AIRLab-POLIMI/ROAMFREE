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
 * MeasurementEdgeWrapperImpl.h
 *
 *  Created on: Mar 13, 2014
 *      Author: davide
 */

#ifndef MEASUREMENTEDGEWRAPPERIMPL_H_
#define MEASUREMENTEDGEWRAPPERIMPL_H_

#include <Eigen/Dense>

#include "MeasurementEdgeWrapper.h"

namespace ROAMestimation {

class BaseEdgeInterface;
class FactorGraphFilter_Impl;

class MeasurementEdgeWrapper_Impl : public MeasurementEdgeWrapper {

	friend FactorGraphFilter_Impl;

public:
	MeasurementEdgeWrapper_Impl(BaseEdgeInterface *edge);
	virtual ~MeasurementEdgeWrapper_Impl() {};

	virtual const Eigen::VectorXd &getMeasurement() const; 
	virtual void setMeasurement(const Eigen::VectorXd &x);

	virtual bool predict();

	virtual PoseVertexWrapper_Ptr getConnectedPose(int N);

protected:
	BaseEdgeInterface *_e;
};

} /* namespace ROAMestimation */

#endif /* MEASUREMENTEDGEWRAPPERIMPL_H_ */
