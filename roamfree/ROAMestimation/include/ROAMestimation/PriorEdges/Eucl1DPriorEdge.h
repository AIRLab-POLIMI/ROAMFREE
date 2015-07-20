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
 * Eucl1DPriorEdge.h
 *
 *  Created on: Jun 12, 2013
 *      Author: davide
 */

#ifndef EUCL1DPRIOREDGE_H_
#define EUCL1DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl1DPriorEdge: public ROAMestimation::BasePriorEdge<1,
    GenericVertex<ROAMfunctions::Eucl1DV> > {

public:

  Eucl1DPriorEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;
};

} /* namespace ROAMestimation */
#endif /* EUCL1DPRIOREDGE_H_ */
