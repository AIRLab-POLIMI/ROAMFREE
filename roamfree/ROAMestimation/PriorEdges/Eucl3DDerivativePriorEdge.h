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
 * Eucl3DDerivativePriorEdge.h
 *
 *  Created on: May 7, 2015
 *      Author: davide
 */

#ifndef EUCL3DDERIVATIVEPRIOREDGE_H_
#define EUCL3DDERIVATIVEPRIOREDGE_H_

#include "BaseDerivativePriorEdge.h"

#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl3DDerivativePriorEdge: public ROAMestimation::BaseDerivativePriorEdge<
    3, GenericVertex<ROAMfunctions::Eucl3DV> > {

  public:

    Eucl3DDerivativePriorEdge();

    void computeError();
    void linearizeOplus();

    std::string writeDebugInfo() const;

};

} /* namespace ROAMestimation */
#endif /* EUCL3DDERIVATIVEPRIOREDGE_H_ */
