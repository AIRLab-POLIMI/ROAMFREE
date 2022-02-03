
/*
 * Eucl2DPriorEdge.h
 *
 *  Created on: Nov 28, 2018
 *      Author: Kenneth J Paul
 */

#ifndef EUCL2DPRIOREDGE_H_
#define EUCL2DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl2DPriorEdge: public ROAMestimation::BasePriorEdge<2,
    GenericVertex<ROAMfunctions::Eucl2DV> > {

public:

  Eucl2DPriorEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;

};

} /* namespace ROAMlog */
#endif /* EUCL2DPRIOREDGE_H_ */
