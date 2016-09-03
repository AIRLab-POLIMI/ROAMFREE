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
 * SE3InterpolationEdge.h
 *
 *  Created on: Aug 31, 2016
 *      Author: davide
 */

#ifndef SE3INTERPOLATIONEDGE_H_
#define SE3INTERPOLATIONEDGE_H_

#include <Eigen/Dense>

#include "g2o/core/base_multi_edge.h"

namespace ROAMestimation {

class SE3InterpolationEdge: public g2o::BaseMultiEdge<6, Eigen::VectorXd> {

  public:
    SE3InterpolationEdge();

    void init();

    void computeError();
    void linearizeOplus();

    std::string writeDebugInfo() const;

    bool read(std::istream &s);
    bool write(std::ostream &s) const;

  protected:
    static const int _OFF = -1;
};

}

#endif /* SE3INTERPOLATIONEDGE_H_ */
