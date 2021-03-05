
/*
 * Eucl4DPriorEdge.h
 *
 *  Created on: Jul 17, 2019
 *      Author: Kenneth J Paul
 */

#ifndef EUCL4DPRIOREDGE_H_
#define EUCL4DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl4DPriorEdge: public ROAMestimation::BasePriorEdge<4,
    GenericVertex<ROAMfunctions::Eucl4DV> > {

public:

  Eucl4DPriorEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;

};

} /* namespace ROAMlog */
#endif /* EUCL4DPRIOREDGE_H_ */
