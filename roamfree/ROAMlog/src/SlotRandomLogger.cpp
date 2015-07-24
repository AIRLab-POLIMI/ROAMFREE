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
 * SlotRandomLogger.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: davide
 */

#include "SlotRandomLogger.h"

#include "ROAMestimation/ParameterTemporaries.h"

namespace ROAMlog {

SlotRandomLogger::SlotRandomLogger(const std::string& file, int prec) :
    _freeSlot(0), _f(file), _precision(prec), _issue(0) {

  _f.precision(_precision);
  _f << std::scientific;

}

SlotRandomLogger::~SlotRandomLogger() {
  _f.close();
}

template<>
int SlotRandomLogger::getToLogVectorSize(
    ROAMestimation::GenericVertexInterface& o) const {

	g2o::OptimizableGraph::Vertex *v = o.getg2oOptGraphPointer();

	int d = v->dimension();

  return v->estimateDimension() + d*(d+1)/2;
}

template<>
int SlotRandomLogger::getToLogVectorSize(
    ROAMestimation::GenericEdgeInterface& o) const {

  g2o::OptimizableGraph::Edge &ov = o;

  int paramsize = 0;
  const std::vector<ROAMestimation::ParameterTemporaries> &params = o.getParameterTemporariesVector();
  for (int k = 0; k<params.size(); k++) {
    paramsize += params[k].value.rows();
  }

  return 1 + 19 + ov.measurementDimension() + paramsize + ov.dimension();
}

template<>
int SlotRandomLogger::getToLogVectorSize(
    ROAMestimation::BasePriorEdgeInterface& o) const {

  g2o::OptimizableGraph::Edge *ov = o.getg2oOptGraphPointer();

  return 1 + ov->dimension();
}

template<>
double SlotRandomLogger::getToLogComponent(int i,
    ROAMestimation::GenericVertexInterface& o) {

	g2o::OptimizableGraph::Vertex *v = o.getg2oOptGraphPointer();

  int n0 = v->estimateDimension();

  if (i < n0) {
  	return v->accessEstimateData()[i];
  } else {
  	int k = i-n0;

  	int toSub = v->dimension();
  	int row = 0;
  	while(k >= toSub) {
  		k-=toSub;
  		toSub--;
  		row++;
  	}
  	int col = row+k;

  	return v->uncertaintyData()[col*v->dimension() + row];
  }
}

template<>
double SlotRandomLogger::getToLogComponent(int i,
    ROAMestimation::GenericEdgeInterface& o) {

  g2o::OptimizableGraph::Edge &ov = o;

  int n0 = 1;
  int n1 = n0 + 19;
  int n2 = n1 + ov.measurementDimension();
  int n3 = n2 + ov.dimension();

  if (i < n0) {
    return o.getFrameCounter();
  } else if (i < n1) {
    return o.getAugmentedState()[i-n0];
  } else if (i < n2) {
    return ov.accessMeasurementData()[i-n1];
  } else if (i < n3) {
  	g2o::OptimizableGraph::Edge *oe = o;

  	// if the edge is robustified we have to undo the process before storing the result
  	if (oe->robustKernel() == true) {
  		return ov.errorData()[i-n2]/oe->currentHuberWeight();
  	} else {
  		return ov.errorData()[i-n2];
  	}
  } else {
    const std::vector<ROAMestimation::ParameterTemporaries> &params = o.getParameterTemporariesVector();

    int pos = i-n3;
    int k = -1;
    int accsize = 0;

    do {
      k++;
      accsize += params[k].value.rows();
    } while (pos >= accsize);

    return params[k].value(pos - (accsize - params[k].value.rows()));
  }
}

template<>
double SlotRandomLogger::getToLogComponent(int i,
    ROAMestimation::BasePriorEdgeInterface& o) {

  g2o::OptimizableGraph::Edge *oe = o.getg2oOptGraphPointer();

  int n0 = 1;
  int n1 = n0 + oe->dimension();

  if (i < n0) {
    return 0; //TODO; there is no frameCounter for prior edges
    //return oe->getFrameCounter();
  } else {
    // if the edge is robustified we have to undo the process before storing the result
    if (oe->robustKernel() == true) {
      return oe->errorData()[i-n0]/oe->currentHuberWeight();
    } else {
      return oe->errorData()[i-n0];
    }
  }
}

} /* namespace ROAMlog */
