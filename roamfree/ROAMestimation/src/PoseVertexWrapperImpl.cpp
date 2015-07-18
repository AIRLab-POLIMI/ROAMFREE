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
 * PoseVertexWrapperImpl.cpp
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#include "PoseVertexWrapperImpl.h"

#include "PoseVertexMetadata.h"

namespace ROAMestimation {

PoseVertexWrapper_Impl::PoseVertexWrapper_Impl(
		g2o::OptimizableGraph::Vertex *vertex) {

#	ifdef DEBUG_BUILD
	_v = dynamic_cast< GenericVertex<ROAMfunctions::SE3V> *>(vertex);
	assert (_v != NULL);
# else
	_v = static_cast< GenericVertex<ROAMfunctions::SE3V> *>(vertex);
# endif
}

} /* namespace ROAMestimation */
