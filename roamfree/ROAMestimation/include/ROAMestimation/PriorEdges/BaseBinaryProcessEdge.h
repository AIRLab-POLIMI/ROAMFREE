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
 * BaseBinaryProcessEdge.h
 *
 *  Created on: Jul 24, 2015
 *      Author: davide
 */

#ifndef BASEBINARYPROCESSEDGE_H_
#define BASEBINARYPROCESSEDGE_H_

#include "g2o/core/base_binary_edge.h"

#include "BasePriorEdgeInterface.h"

namespace ROAMestimation {

template<int D, typename VertexXi>
class BaseBinaryProcessEdge: public g2o::BaseBinaryEdge<D, Eigen::VectorXd,
    VertexXi, VertexXi>, public BasePriorEdgeInterface {

  protected:

    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_measurement;
    using g2o::BaseBinaryEdge<D, Eigen::VectorXd, VertexXi, VertexXi>::_information;

    Eigen::Matrix<double, D, D> _noiseCov;

    std::string _name;
    double _tstamp;

  public:

    virtual ~BaseBinaryProcessEdge() {
    }

    virtual void setNoiseCov(const Eigen::MatrixXd &noiseCov) {
      _noiseCov = noiseCov;
    }

    virtual void setMeasurement(const Eigen::VectorXd &measurement) {
      std::cerr << "[BaseBinaryProcessEdge] Warning: measurement plays no role here " << std::endl;

      _measurement = measurement;
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

};

}

#endif /* BASEBINARYPROCESSEDGE_H_ */

