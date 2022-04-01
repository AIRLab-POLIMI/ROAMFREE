/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Kenneth Joseph Paul (kenneth.josephpaul@epfl.ch)    
*/

/*
 * Eucl2DGaussMarkovProcessEdge.cpp
 *
 *  Created on: July 27, 2021
 *      Author: kenneth
 */

#ifndef EUCL2DGAUSSMARKOVPROCESSEDGE_H_
#define EUCL2DGAUSSMARKOVPROCESSEDGE_H_

#include "BaseGaussMarkovProcessEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl2DGaussMarkovProcessEdge: public ROAMestimation::BaseGaussMarkovProcessEdge<
    2, GenericVertex<ROAMfunctions::Eucl2DV> > {

  public:

    Eucl2DGaussMarkovProcessEdge();

    void computeError();
    void linearizeOplus();

    std::string writeDebugInfo() const;

};

} /* namespace ROAMestimation */
#endif /* EUCL2DGAUSSMARKOVPROCESSEDGE_H_ */
