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
 * PoseVertexMetadata.h
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#ifndef POSEVERTEXMETADATA_H_
#define POSEVERTEXMETADATA_H_

#include "g2o/core/optimizable_graph.h"

namespace ROAMestimation {

/**
 * container for Pose vertex metadata, to be stored in the userData field of
 * g2o::OptimizableGraph::Vertex *
 */

struct PoseVertexMetadata : public g2o::OptimizableGraph::Data {

	bool computeUncertainty;
	bool keepSpatialIndex;
	bool hasBeenEstimated;

	PoseVertexMetadata() : computeUncertainty(false), keepSpatialIndex(false), hasBeenEstimated(false) {}

	//! read from a stream not implemented
  inline virtual bool read(std::istream& is) {
  	return false;
  }
  //! write to a stream not implemented
  inline virtual bool write(std::ostream& os) const {
  	return false;
  }
};

}

#endif /* POSEVERTEXMETADATA_H_ */
