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
 * SE3Prior.h
 *
 *  Created on: Apr 8, 2013
 *      Author: davide
 */

#ifndef SE3PRIOREDGE_H_
#define SE3PRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class SE3PriorEdge: public ROAMestimation::BasePriorEdge<6, GenericVertex<ROAMfunctions::SE3V> > {

public:

  SE3PriorEdge();

  void afterVertexUpdate();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;

protected:
  static const int _OFF = -1;
  double sign;
};

}
/* namespace ROAMestimation */
#endif /* SE3PRIOR_H_ */
