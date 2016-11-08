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
 * MeasurementEdgeWrapperImpl.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: davide
 */

#include "MeasurementEdgeWrapperImpl.h"
#include "PoseVertexWrapperImpl.h"

#include "GenericEdgeInterface.h"

namespace ROAMestimation {

MeasurementEdgeWrapper_Impl::MeasurementEdgeWrapper_Impl(
    GenericEdgeInterface* edge) :
    _e(edge) {
}

bool MeasurementEdgeWrapper_Impl::predict() {
  //TODO: there is an assert in the predictNextState method that make everything crash if the edge does not support prediction
  _e->predictNextState();
  return true;
}

PoseVertexWrapper_Ptr MeasurementEdgeWrapper_Impl::getConnectedPose(
    int N)
    {
  if (N > _e->getOrder()) {
    return PoseVertexWrapper_Ptr(NULL);
  }

  g2o::OptimizableGraph::Edge *oe = _e->getg2oOptGraphPointer();
  return PoseVertexWrapper_Ptr(
      new PoseVertexWrapper_Impl(
          static_cast<g2o::OptimizableGraph::Vertex *>(oe->vertices()[N])));
}

} /* namespace ROAMestimation */
