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
 * GenericEdge.h
 *
 *  Created on: Nov 7, 2013
 *      Author: davide
 */

#ifndef GENERICEDGE_H_
#define GENERICEDGE_H_

#include "g2o/core/base_multi_edge.h"

#include "GenericEdgeInterface.h"

#include "ROAMmath/MatricesOperations.h"

namespace ROAMestimation {

/** \brief generic ROAMFREE edge
 *  - error and measurement storage type Egein::VectorXd
 *  - it has a _noiseCov field which holds the covariance of the noise vector
 *  - ROAMFREE edges have parameters, temporaries for them are held in the _param field
 *  - they have a Category (a name), and a timestamp
 *
 */

template<int D>
class GenericEdge: public g2o::BaseMultiEdge<D, Eigen::VectorXd>,
    public GenericEdgeInterface {

  public:

    GenericEdge(int nParams);
    virtual ~GenericEdge();

    virtual void setNoiseCov(const Eigen::MatrixXd &noiseCov);
    virtual const Eigen::MatrixXd & getNoiseCov() const;

    virtual const Eigen::VectorXd& getMeasurement() const;
    virtual void setMeasurement(const Eigen::VectorXd& m);

    virtual void handleParamTemporaries();
    virtual void updateParamPtrs();
    virtual const std::vector<ParameterTemporaries> & getParameterTemporariesVector() const;

    virtual void setTimestamp(double timestamp);
    virtual double getTimestamp() const;

    virtual void setCategory(const std::string &name);
    virtual const std::string &getCategory() const;

    virtual const Eigen::VectorXd &getAugmentedState() const;

    inline virtual g2o::OptimizableGraph::Edge *getg2oOptGraphPointer() {
      return static_cast<g2o::OptimizableGraph::Edge *>(this);
    }

    virtual bool read(std::istream &s);
    virtual bool write(std::ostream &s) const;

  protected:

    std::string _name; /**< the name of the sensor this edge refers to */

    double _tstamp; /**< timestamp of the measurement */

    double _Dt01, _Dt12; /**< dt between pose vertices */

    long int _frameCounter;

    Eigen::VectorXd _x; /**< augmented state: [x, q, v, omega, a, alpha,. disp, dispQ, IMUintdP, IMUintdQ], 33 variables */

    Eigen::MatrixXd _noiseCov; /**< noise covariance matrix, it is not necessarily the inverse of _information
     *                               i.e. the noise vector and the measurement vector have different sizes */

    std::vector<ParameterTemporaries> _params; /**< data structure which describe parameters and local temporary values */
    int _paramsPtrsSize; /**< size of the parameter pointer arrays, it is the number of non standard parameters of the edge */
    double **_paramsPtrs; /**< shortcuts to memory where current interpolated parameter estimate is stored */

    using g2o::BaseMultiEdge<D, Eigen::VectorXd>::_information;
    using g2o::BaseMultiEdge<D, Eigen::VectorXd>::_measurement;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ;
};

#define USINGDIRECTIVES using GenericEdge<MT::_ERROR_SIZE>::_noiseCov;      \
using GenericEdge<MT::_ERROR_SIZE>::_params;                                \
using GenericEdge<MT::_ERROR_SIZE>::_paramsPtrs;                            \
using GenericEdge<MT::_ERROR_SIZE>::_x;                                     \
\
using GenericEdge<MT::_ERROR_SIZE>::handleParamTemporaries;                 \
using GenericEdge<MT::_ERROR_SIZE>::_name;                                  \
using GenericEdge<MT::_ERROR_SIZE>::_tstamp;                                \
using GenericEdge<MT::_ERROR_SIZE>::_Dt01;                                  \
using GenericEdge<MT::_ERROR_SIZE>::_Dt12;                                  \
using GenericEdge<MT::_ERROR_SIZE>::_frameCounter;                          \
\
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::_information;   \
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::_measurement;   \
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::_error;         \
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::_vertices;      \
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::_jacobianOplus; \
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::chi2; \
\
using g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::resize;

template<int D>
GenericEdge<D>::GenericEdge(int nParams) :
    _paramsPtrsSize(nParams), _name("undefined"), _tstamp(
        -std::numeric_limits<double>::infinity()), _Dt01(0), _Dt12(0), _x(33), _frameCounter(
        -1) {

  _paramsPtrs = new double *[_paramsPtrsSize]; // 1 Euclidean 3D, 1 Quaternion the possible function specific params
}

template<int D>
inline GenericEdge<D>::~GenericEdge() {
  delete[] _paramsPtrs;
}

template<int D>
void GenericEdge<D>::setNoiseCov(
    const Eigen::MatrixXd& noiseCov) {

  _noiseCov = noiseCov;

  // if these matrices match in size initialize the information matrix
  // with the inverse of the covariance matrix
  // if there are jacobians involved it will be fixed later
  if (_noiseCov.rows() == _information.rows()) {
    ROAMmath::inv(_noiseCov, _information);
  }
}

template<int D>
inline const Eigen::MatrixXd& GenericEdge<D>::getNoiseCov() const {
  return _noiseCov;
}

template<int D>
inline void GenericEdge<D>::handleParamTemporaries() {

  for (auto it = _params.begin(); it != _params.end(); ++it) {
    it->updateTemporaries();
  }

  // TODO: this in principle could be done only once, there is a bug somewhere that make these pointers change sometimes
  updateParamPtrs();
}

template<int D>
void GenericEdge<D>::updateParamPtrs() {
  for (int k = 0; k < _paramsPtrsSize; k++) {
    _paramsPtrs[k] = _params[k + 2].value.data();
  }
}

template<int D>
inline const std::vector<ParameterTemporaries>& GenericEdge<
    D>::getParameterTemporariesVector() const {
  return _params;
}

template<int D>
inline bool GenericEdge<D>::read(std::istream& s) {
  return false;
}

template<int D>
inline bool GenericEdge<D>::write(std::ostream& s) const {
  return false;
}

template<int D>
inline void GenericEdge<D>::setTimestamp(double timestamp) {
  _tstamp = timestamp;
}

template<int D>
inline double GenericEdge<D>::getTimestamp() const {
  return _tstamp;
}

template<int D>
inline void GenericEdge<D>::setCategory(
    const std::string& name) {
  _name = name;
}

template<int D>
inline const std::string& GenericEdge<D>::getCategory() const {
  return _name;
}

template<int D>
inline const Eigen::VectorXd& GenericEdge<D>::getMeasurement() const {
  return _measurement;
}

template<int D>
inline void GenericEdge<D>::setMeasurement(
    const Eigen::VectorXd& m) {
  _measurement = m;
}

template<int D>
inline const Eigen::VectorXd& GenericEdge<D>::getAugmentedState() const {
  return _x;
}

} /* namespace ROAMestimation */

#endif /* GENERICEDGE_H_ */
