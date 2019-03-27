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
 * QuaternionPriorEdge.h
 *
 *  Created on: Mar 26, 2019
 *      Author: davide
 */

#ifndef QUATERNIONPRIOREDGE_H_
#define QUATERNIONPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class QuaternionPriorEdge: public ROAMestimation::BasePriorEdge<3, GenericVertex<ROAMfunctions::QuaternionV> > {

public:

  QuaternionPriorEdge();

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
#endif /* QUATERNIONPRIOREDGE_H_ */
