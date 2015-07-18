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
 * MeasurementEdgeMetadata.h
 *
 *  Created on: Mar 14, 2014
 *      Author: davide
 */

#ifndef MEASUREMENTEDGEMETADATA_H_
#define MEASUREMENTEDGEMETADATA_H_

#include "g2o/core/optimizable_graph.h"

namespace ROAMestimation {

/**
 * container for Pose vertex metadata, to be stored in the userData field of
 * g2o::OptimizableGraph::Vertex *
 */

struct MeasurementEdgeMetadata : public g2o::OptimizableGraph::Data {

	bool insertedAterLastEstimate;

	MeasurementEdgeMetadata() : insertedAterLastEstimate(true) { }

	//! read from a stream not implemented
  inline virtual bool read(std::istream& is) {
  	return false;
  }
  //! write to a stream not implemented
  inline virtual bool write(std::ostream& os) const {
  	return false;
  }

};

} /* namespace ROAMestimation */

#endif /* MEASUREMENTEDGEMETADATA_H_ */
