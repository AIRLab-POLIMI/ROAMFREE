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
 * Matrix3DPriorEdge.h
 *
 *  Created on: Jul 11, 2013
 *      Author: davide
 */

#ifndef MATRIX3DPRIOREDGE_H_
#define MATRIX3DPRIOREDGE_H_

#include "BasePriorEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Matrix3DPriorEdge: public ROAMestimation::BasePriorEdge<9,
GenericVertex<ROAMfunctions::Matrix3DV> > {

public:

Matrix3DPriorEdge();

void computeError();
void linearizeOplus();

std::string writeDebugInfo() const;

};

} /* namespace ROAMestimation */
#endif /* MATRIX3DPRIOREDGE_H_ */
