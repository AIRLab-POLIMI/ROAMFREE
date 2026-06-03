
/*
 * Eucl5DPriorEdge.h
 *
 *  Created on: Apr 22, 2026
 *      Author: Simon Gilgien
 */

#ifndef EUCL5DPRIOREDGE_H_
#define EUCL5DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl5DPriorEdge: public ROAMestimation::BasePriorEdge<5,
    GenericVertex<ROAMfunctions::Eucl5DV> > {

public:

  Eucl5DPriorEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;

};

} /* namespace ROAMlog */
#endif /* EUCL5DPRIOREDGE_H_ */
