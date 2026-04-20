
/*
 * Eucl6DPriorEdge.h
 *
 *  Created on: Apr 20, 2026
 *      Author: Simon Gilgien
 */

#ifndef EUCL6DPRIOREDGE_H_
#define EUCL6DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl6DPriorEdge: public ROAMestimation::BasePriorEdge<6,
    GenericVertex<ROAMfunctions::Eucl6DV> > {

public:

  Eucl6DPriorEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;

};

} /* namespace ROAMlog */
#endif /* EUCL6DPRIOREDGE_H_ */
