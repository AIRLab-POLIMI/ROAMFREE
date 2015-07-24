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
 * BasePrior.h
 *
 *  Created on: Jun 12, 2013
 *      Author: davide
 */

#ifndef BASEPRIOR_H_
#define BASEPRIOR_H_

#include <iomanip>

#include <Eigen/Dense>
#include "ROAMmath/MatricesOperations.h"

#include "g2o/core/base_unary_edge.h"

#include "BasePriorEdgeInterface.h"

namespace ROAMestimation {

template<int D, typename VertexXi>

class BasePriorEdge: public g2o::BaseUnaryEdge<D, Eigen::VectorXd, VertexXi>,
    public BasePriorEdgeInterface {

  protected:

    using g2o::BaseUnaryEdge<D, Eigen::VectorXd, VertexXi>::_measurement;
    using g2o::BaseUnaryEdge<D, Eigen::VectorXd, VertexXi>::_information;

    Eigen::Matrix<double, D, D> _noiseCov;

    std::string _name;
    double _tstamp;

  public:

    virtual ~BasePriorEdge() {
    }

    virtual void setNoiseCov(const Eigen::MatrixXd &noiseCov) {
      _noiseCov = noiseCov;

      // TODO: this causes an extra inversion for those edges which have non-identity jacobian wrt error
      ROAMmath::inv(_noiseCov, _information);
    }

    virtual void setMeasurement(const Eigen::VectorXd &measurement) {
      _measurement = measurement;
    }

    virtual g2o::OptimizableGraph::Edge *getg2oOptGraphPointer() {
      return static_cast<g2o::OptimizableGraph::Edge *>(this);
    }

    virtual void setCategory(const std::string &name) {
      _name = name;
    }

    virtual const std::string &getCategory() const {
      return _name;
    }

    virtual void setTimestamp(double timestamp) {
      _tstamp = timestamp;
    }

    virtual double getTimestamp() const {
      return _tstamp;
    }

    bool read(std::istream &s) {
      return false;
    }

    bool write(std::ostream &s) const {
      return false;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace ROAMestimation */
#endif /* BASEPRIOR_H_ */
