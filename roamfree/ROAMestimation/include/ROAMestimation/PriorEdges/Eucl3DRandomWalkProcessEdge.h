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
 * Eucl3DRandomWalkProcessEdge.h
 *
 *  Created on: Jul 31, 2015
 *      Author: davide
 */

#ifndef EUCL3DRANDOMWALKPROCESSEDGE_H_
#define EUCL3DRANDOMWALKPROCESSEDGE_H_

#include "BaseRandomWalkProcessEdge.h"
#include "GenericVertex.h"

namespace ROAMestimation {

class Eucl3DRandomWalkProcessEdge: public ROAMestimation::BaseRandomWalkProcessEdge<
    3, GenericVertex<ROAMfunctions::Eucl3DV> > {

  public:

    Eucl3DRandomWalkProcessEdge();

    void computeError();
    void linearizeOplus();

    std::string writeDebugInfo() const;

};

}
#endif /* EUCL3DRANDOMWALKPROCESSEDGE_H_ */
