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


#ifndef EUCL2DRANDOMWALKPROCESSEDGE_H_
#define EUCL2DRANDOMWALKPROCESSEDGE_H_

#include "BaseRandomWalkProcessEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl2DRandomWalkProcessEdge: public ROAMestimation::BaseRandomWalkProcessEdge<
    2, GenericVertex<ROAMfunctions::Eucl2DV> > {

  public:

    Eucl2DRandomWalkProcessEdge();

    void computeError();
    void linearizeOplus();

    std::string writeDebugInfo() const;

};

}
#endif /* EUCL2DRANDOMWALKPROCESSEDGE_H_ */
