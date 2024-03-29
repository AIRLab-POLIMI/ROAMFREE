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
 * MeasurementEdgeWrapper.h
 *
 *  Created on: Mar 13, 2014
 *      Author: davide
 */

#ifndef MEASUREMENTEDGEWRAPPER_H_
#define MEASUREMENTEDGEWRAPPER_H_

#include "interfaceTypes.h"

namespace ROAMestimation {

class MeasurementEdgeWrapper {
public:

  virtual ~MeasurementEdgeWrapper() {};

	virtual const Eigen::VectorXd &getMeasurement() const = 0; //!< get a reference to the measurement
	virtual void setMeasurement(const Eigen::VectorXd &x) = 0; //!< set the measurement

	/*
	 * \brief evaluates a prediction for the most recent pose in the edge
	 *
	 * The estimate for the  most recent pose involved in the edge is replaced
	 * with a prediction computed with the edge measurement model based
	 * on the measurement and possibly on the older poses (if they exist)
	 *
	 * @return false if the edge does not support prediction
	 */
	virtual bool predict() = 0;

	/*
	 * \ brief get a wrapper to the pose vertices that are connected by this edge
	 */

	virtual PoseVertexWrapper_Ptr getConnectedPose(int N) = 0;
};

} /* namespace ROAMestimation */

#endif /* MEASUREMENTEDGEWRAPPER_H_ */
