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
 * PoseVertexWrapper.h
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#ifndef POSEVERTEXWRAPPER_H_
#define POSEVERTEXWRAPPER_H_

#include <Eigen/Dense>

#include "interfaceTypes.h"

namespace ROAMestimation {

/**
 * This class wraps a Pose vertex
 */

class PoseVertexWrapper {

public:
	virtual ~PoseVertexWrapper() {};

	virtual double getTimestamp() const = 0; //!< returns the current timestamp for this node
	virtual void setTimestamp(double t) = 0; //!< sets the current timestamp. Might help in vertex initialization. Do not change timestamps while running

	virtual bool getFixed() const = 0; //!< returns if the vertex is fixed or not during the estimation
	virtual void setFixed(bool isFixed) = 0; //!< sets if the vertex is fixed or not during the estimation

	virtual const Eigen::VectorXd &getEstimate() const = 0; //!< get a reference to the current estimate
	virtual void setEstimate(const Eigen::VectorXd &x) = 0; //!< set the current estimate

	virtual bool getComputeUncertainty() const = 0; //!< get if the uncertainty for this vertex has to be computed after estimation
	virtual void setComputeUncertainty(bool computeUncertainty) = 0; //!< set if the uncertainty for this vertex has to be computed after estimation

	virtual const Eigen::Matrix<double,6,6> &getUncertainty() const = 0; //!< get the covariance matrix encoding the uncertainty about this vertex estimate

	virtual bool getKeepSpatialIndex() const = 0; //!< get if this node has to be kept into the spatial index
	virtual void setKeepSpatialIndex(bool keepSpatialIndex) = 0; //!< set if this node has to be kept into the spatial index

	virtual bool hasBeenEstimated() const = 0; //!< get if this node already participated in an estimation

	virtual bool sameVertexAs(PoseVertexWrapper_Ptr other) = 0; //!< returns true if the vertex wrapped by other is the same as mine
	virtual int getId() const = 0; //!< returns the unique id of the vertex

};


} /* namespace ROAMestimation */

#endif /* POSEVERTEXWRAPPER_H_ */
