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
 * BaseGaussMarkovProcessEdge.h
 *
 *  Created on: May 7, 2015
 *      Author: davide
 *
 *      constraint the difference between two vertices
 *      second = exp(-beta *dt) * first + noise
 *
 *      measurement plays no role here.
 *
 */

#ifndef BASEGAUSSMARKOVPROCESSEDGE_H_
#define BASEGAUSSMARKOVPROCESSEDGE_H_

#include <Eigen/Dense>
#include "ROAMmath/MatricesOperations.h"

#include "g2o/core/base_binary_edge.h"

#include "BaseBinaryProcessEdge.h"

namespace ROAMestimation {

template<int D, typename VertexXi>

class BaseGaussMarkovProcessEdge: public BaseBinaryProcessEdge<D, VertexXi> {

  protected:

    Eigen::Matrix<double, D, 1> _beta;
    double _dt;

    using BaseBinaryProcessEdge<D, VertexXi>::_noiseCov;

    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_information;

    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_jacobianOplusXj;
    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_jacobianOplusXi;

  public:

    virtual ~BaseGaussMarkovProcessEdge() {
    }

    virtual void init(const Eigen::VectorXd &beta, double dt) {
      assert(beta.rows() == D);
      _beta = beta;
      _dt = dt;

      _jacobianOplusXj = Eigen::Matrix3d::Identity(); // second ...

      _jacobianOplusXi.setZero(); // -exp(beta dt) * first
      for (int i = 0; i<D; i++) {
        _jacobianOplusXi(i,i) = -exp(-_beta(i)*_dt);
      }

      // obtain the discrete time covariance matrix as a function
      // of beta and dt

      for (int i = 0; i<D; i++) {
        assert(_beta(i) >= 0);

        if (_beta(i) != 0) {
          _noiseCov(i,i) *= 1.0/(2.0*_beta(i))*(1.0 - exp(-2.0*_beta(i)*_dt));
        } else {
          _noiseCov(i,i) *= _dt;
        }
      }

      ROAMmath::inv(_noiseCov, _information);
    }

    virtual g2o::OptimizableGraph::Edge *getg2oOptGraphPointer() {
      return static_cast<g2o::OptimizableGraph::Edge *>(this);
    }

};

} /* namespace ROAMestimation */

#endif /* BASEGAUSSMARKOVPROCESSEDGE_H_ */
