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
 * Eucl3DPriorEdge.h
 *
 *  Created on: Jun 12, 2013
 *      Author: davide
 */

#ifndef EUCL3DPRIOREDGE_H_
#define EUCL3DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl3DPriorEdge: public ROAMestimation::BasePriorEdge<3,
    GenericVertex<ROAMfunctions::Eucl3DV> > {

public:

  Eucl3DPriorEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;

};

} /* namespace ROAMlog */
#endif /* EUCL3DPRIOREDGE_H_ */
