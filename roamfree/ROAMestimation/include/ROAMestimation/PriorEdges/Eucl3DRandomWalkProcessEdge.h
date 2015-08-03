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
