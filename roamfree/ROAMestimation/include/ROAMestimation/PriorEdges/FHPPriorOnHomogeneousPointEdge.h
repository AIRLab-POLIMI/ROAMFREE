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
 * FHPPriorOnHomogeneousPoint.h
 *
 *  Created on: Dec 2, 2014
 *      Author: davide
 */

#ifndef FHPPRIORONHOMOGENEOUSPOINT_H_
#define FHPPRIORONHOMOGENEOUSPOINT_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class FHPPriorOnHomogeneousPointEdge: public ROAMestimation::BasePriorEdge<2,
    GenericVertex<ROAMfunctions::Eucl3DV> > {

public:

	FHPPriorOnHomogeneousPointEdge();

  void computeError();
  void linearizeOplus();

  std::string writeDebugInfo() const;
};

} /* namespace ROAMestimation */
#endif /* FHPPRIORONHOMOGENEOUSPOINT_H_ */
