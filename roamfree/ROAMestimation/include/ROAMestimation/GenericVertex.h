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
 * CinematicGenericVertex.h
 *
 *  Created on: Dec 17, 2012
 *      Author: davide
 */

#ifndef CINEMATICGENERICVERTEX_H_
#define CINEMATICGENERICVERTEX_H_

#include <iomanip>
#include <Eigen/Dense>

#include "g2o/core/base_vertex.h"
#include "g2o/core/viewer_vertex_decorator.h"
#include "GenericVertexInterface.h"

#include "ROAMfunctions/AllVariables.h"

namespace ROAMestimation {

template<typename MT>
class GenericVertex: public g2o::BaseVertex<
    ROAMfunctions::VariableTraits<MT>::_INCREMENT_SIZE, Eigen::VectorXd>,
    public g2o::ViewerVertexDecorator,
    public GenericVertexInterface {

  protected:

    typedef ROAMfunctions::VariableTraits<MT> Traits;

    // real object implementing oplus operator
    typename Traits::VariableI _V;

    // using stuff
    using g2o::BaseVertex<Traits::_INCREMENT_SIZE, Eigen::VectorXd>::_estimate;
    using g2o::BaseVertex<Traits::_INCREMENT_SIZE, Eigen::VectorXd>::_dimension;
    using g2o::BaseVertex<Traits::_INCREMENT_SIZE, Eigen::VectorXd>::_fixed;
    using g2o::BaseVertex<Traits::_INCREMENT_SIZE, Eigen::VectorXd>::_uncertainty;

    bool _gtKnown;
    Eigen::VectorXd _gt;

    bool _hasTimestamp;
    double _timestamp;

    std::string _category;

    // initializes internal structures, needed in constructor
    void init() {
      _estimate.resize(Traits::_INTERNAL_SIZE);
      _gt.resize(Traits::_INTERNAL_SIZE);
      _gtKnown = false;
      _hasTimestamp = false;

      //initialize the cell 0,0 of the covariance matrix to +inf so I can distinguish
      //the vertices for which I have computed the covariance
      _uncertainty(0, 0) = std::numeric_limits<double>::infinity();
    }

  public:

    GenericVertex() {
      init();
    }
    GenericVertex(const GenericVertex<MT> &v) :
        ViewerVertexDecorator(v) {
      init();

      _estimate = v.estimate();
      _gtKnown = v.hasGT();
      if (_gtKnown) {
        _gt = v.accessGt();
      }
      _hasTimestamp = v.hasTimestamp();
      if (_hasTimestamp) {
        _timestamp = v.getTimestamp();
      }

    }

    virtual ~GenericVertex() {
    }

    void oplus(double *dx) {
      // wrap dx into and Eigen::VectorXd
      Eigen::VectorXd tmp_dx = Eigen::Map<Eigen::VectorXd>(dx,
          Traits::_INCREMENT_SIZE, 1);

      // call the method of _V which applies dx
      _V.oplus(_estimate, tmp_dx);
    }

    void setToOrigin() {
      _V.setToOrigin(_estimate);
    }

    virtual int estimateDimension() const {
      return Traits::_INTERNAL_SIZE;
    }

    virtual bool getEstimateData(double* estimate) const {
      for (unsigned int k = 0; k < estimateDimension(); k++) {
        estimate[k] = _estimate(k);
      }
      return true;
    }

    virtual const Eigen::VectorXd &getEstimate() const {
      return _estimate;
    }

    virtual void setEstimate(const Eigen::VectorXd &x) {
      _estimate = x;
    }

    double *accessEstimateData() {
      return _estimate.data();
    }

    inline const Eigen::VectorXd &accessGt() const {
      return _gt;
    }

    void setGT(const Eigen::VectorXd &gt) {
      _gt = gt;
      _gtKnown = true;
    }

    inline bool hasGT() const {
      return _gtKnown;
    }

    void setTimestamp(double timestamp) {
      _timestamp = timestamp;
      _hasTimestamp = true;
    }

    inline bool hasTimestamp() const {
      return _hasTimestamp;
    }

    inline double getTimestamp() const {
      return _timestamp;
    }

    const std::string &getCategory() const {
      return _category;
    }

    void setCategory(const std::string &category) {
      _category = category;

      // we don't want spaces in this
      std::replace(_category.begin(), _category.end(), ' ', '_');
    }

    virtual g2o::OptimizableGraph::Vertex *getg2oOptGraphPointer() {
      return static_cast<g2o::OptimizableGraph::Vertex *>(this);
    }

    bool write(std::ostream &s) const {
      return false;
    }

    bool read(std::istream &s) {
      return false;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif /* CINEMATICGENERICVERTEX_H_ */
