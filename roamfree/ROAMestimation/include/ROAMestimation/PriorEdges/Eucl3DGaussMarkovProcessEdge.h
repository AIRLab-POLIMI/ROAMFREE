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
 * Eucl3DGaussMarkovProcessEdge.h
 *
 *  Created on: May 7, 2015
 *      Author: davide
 */

#ifndef EUCL3DGAUSSMARKOVPROCESSEDGE_H_
#define EUCL3DGAUSSMARKOVPROCESSEDGE_H_

#include "BaseGaussMarkovProcessEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl3DGaussMarkovProcessEdge: public ROAMestimation::BaseGaussMarkovProcessEdge<
    3, GenericVertex<ROAMfunctions::Eucl3DV> > {

  public:

    Eucl3DGaussMarkovProcessEdge();

    void computeError();
    void linearizeOplus();

    std::string writeDebugInfo() const;

};

} /* namespace ROAMestimation */
#endif /* EUCL3DGAUSSMARKOVPROCESSEDGE_H_ */
