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
 * PoseVertexWrapperImpl.h
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#ifndef POSEVERTEXWRAPPERIMPL_H_
#define POSEVERTEXWRAPPERIMPL_H_

#include "GenericVertex.h"

#include "PoseVertexWrapper.h"
#include "PoseVertexMetadata.h"

#include "types.h"

namespace ROAMestimation {

class FactorGraphFilter_Impl;

class PoseVertexWrapper_Impl: public PoseVertexWrapper {

	friend FactorGraphFilter_Impl;

protected:
	GenericVertex<ROAMfunctions::SE3V> *_v;

public:
	PoseVertexWrapper_Impl(g2o::OptimizableGraph::Vertex *vertex);

	inline virtual double getTimestamp() const {
		return _v->getTimestamp();
	}

	inline void setTimestamp(double t) {
		_v->setTimestamp(t);
	}

	inline virtual bool getFixed() const {
		return _v->fixed();
	}
	inline virtual void setFixed(bool isFixed) {
		_v->setFixed(isFixed);
	}

	inline virtual const Eigen::VectorXd &getEstimate() const {
		return _v->estimate();
	}
	inline virtual void setEstimate(const Eigen::VectorXd &x) {
		_v->estimate() = x;
	}

	inline virtual bool getComputeUncertainty() const {
		const PoseVertexMetadata * meta = static_cast<const PoseVertexMetadata *>(_v->userData());
		return meta->computeUncertainty;
	}
	inline virtual void setComputeUncertainty(bool computeUncertainty) {
		PoseVertexMetadata * meta = static_cast<PoseVertexMetadata *>(_v->userData());
		meta->computeUncertainty = computeUncertainty;
	}

	inline virtual const Eigen::Matrix<double, 6, 6> &getUncertainty() const {
		return _v->uncertainty();
	}

	inline virtual bool getKeepSpatialIndex() const {
		const PoseVertexMetadata * meta = static_cast<const PoseVertexMetadata *>(_v->userData());
		return meta->keepSpatialIndex;
	}
	inline virtual void setKeepSpatialIndex(bool keepSpatialIndex) {
		PoseVertexMetadata * meta = static_cast<PoseVertexMetadata *>(_v->userData());
		meta->keepSpatialIndex = keepSpatialIndex;
	}

	inline bool hasBeenEstimated() const {
	  PoseVertexMetadata * meta = static_cast<PoseVertexMetadata *>(_v->userData());
	  return meta->hasBeenEstimated;
	}

	inline virtual bool sameVertexAs(PoseVertexWrapper_Ptr other) {
		PoseVertex *otherVertex = boost::static_pointer_cast<PoseVertexWrapper_Impl>(other)->_v;

		return otherVertex == _v;
	}

	inline virtual int getId() const {
		return _v->id();
	}

};

} /* namespace ROAMestimation */

#endif /* POSEVERTEXWRAPPERIMPL_H_ */
