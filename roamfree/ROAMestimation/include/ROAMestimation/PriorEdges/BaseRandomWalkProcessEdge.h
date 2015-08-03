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
 * BaseRandomWalkProcessEdge.h
 *
 *  Created on: Jul 31, 2015
 *      Author: davide
 */

#ifndef BASERANDOMWALKPROCESSEDGE_H_
#define BASERANDOMWALKPROCESSEDGE_H_

#include <Eigen/Dense>
#include "ROAMmath/MatricesOperations.h"

#include "g2o/core/base_binary_edge.h"

#include "BaseBinaryProcessEdge.h"

namespace ROAMestimation {

template<int D, typename VertexXi>

class BaseRandomWalkProcessEdge: public BaseBinaryProcessEdge<D, VertexXi> {

  protected:

    double _dt;

    using BaseBinaryProcessEdge<D, VertexXi>::_noiseCov;

    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_information;

    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_jacobianOplusXj;
    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_jacobianOplusXi;

  public:

    virtual ~BaseRandomWalkProcessEdge() {
    }

    virtual void init(double dt) {
      _dt = dt;

      _jacobianOplusXj = Eigen::MatrixXd::Identity(D,D); // newer ...

      _jacobianOplusXi = -Eigen::MatrixXd::Identity(D,D); // - older

      ROAMmath::invDiagonal(_noiseCov, _information);

      for (int i = 0; i< D; i++) {
        _information(i,i) /= _dt;
      }
    }

    virtual g2o::OptimizableGraph::Edge *getg2oOptGraphPointer() {
      return static_cast<g2o::OptimizableGraph::Edge *>(this);
    }

};

} /* namespace ROAMestimation */



#endif /* BASERANDOMWALKPROCESSEDGE_H_ */
